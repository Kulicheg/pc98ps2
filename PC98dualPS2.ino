////////////////////////////////////////////////////////////////////////////////
// PC98dualPS2
// Одна Arduino Nano: PS/2 клавиатура -> клавиатура PC-98 и
// PS/2 мышь -> шинная (bus) мышь PC-98.
//
// Клавиатурные карты: zakernichny/ps2topc98.
// Квадратура шины: tyama501/ps2busmouse98 (открытый коллектор, те же фазы).
//
// Горячий путь не вызывает digitalRead/digitalWrite/pinMode/millis.
// Бит PS/2 снимается с PINx в прерывании по спаду clock.
// Шина мыши: DDR=1 тянет линию в землю, DDR=0 отпускает на внешние 10 кОм.
//
// Питание Nano на пин 5V, не на VIN. +5 В только от одного разъёма PC-98
// (клавиатура pin 8 ИЛИ мышь DB9 pin 1). Земли всех разъёмов вместе.
// При прошивке по USB кабель PC-98 снят, чтобы не связать два источника 5 В.
//
// Распиновка Arduino Nano (ATmega328P, 5 В, 16 МГц)
//
//   D0  PD0   UART RX     не занимать (USB-serial / DEBUG)
//   D1  PD1   UART TX     PC-98 клавиатура, mini-DIN-8 pin 4 (RXD хоста)
//   D2  PD2   INT0        PS/2 мышь clock,    mini-DIN-6 pin 5
//   D3  PD3   INT1        PS/2 клавиатура clock, mini-DIN-6 pin 5
//   D4  PD4   вход        PC-98 ~RTY, mini-DIN-8 pin 5
//   D5  PD5   R           DB9 pin 8, правая кнопка, открытый коллектор
//   D6  PD6   вход        PC-98 ~RST, mini-DIN-8 pin 1
//   D7  PD7   вход        PC-98 ~RDY, mini-DIN-8 pin 3
//   D8  PB0   XA          DB9 pin 2, открытый коллектор + 10 кОм на 5V
//   D9  PB1   XB          DB9 pin 3
//   D10 PB2   YA          DB9 pin 4
//   D11 PB3   YB          DB9 pin 5
//   D12 PB4   L           DB9 pin 6, левая кнопка
//   D13 PB5   не занимать: светодиод платы сажает открытый коллектор
//   A0  PC0               PS/2 мышь data,       mini-DIN-6 pin 1
//   A1  PC1               PS/2 клавиатура data, mini-DIN-6 pin 1
//   A2..A5                свободны
//   A6 A7                 только АЦП, не использовать
//
// DB9 pin 7 не подключать. GND: mini-DIN-8 pin 2, DB9 pin 9, оба PS/2 pin 3.
// +5 В на оба PS/2 pin 4 с той же шины, что питает Nano.
// На clock и data каждой PS/2 по 10 кОм на +5 В.
//
// Сброс DEBUG: лог в USB-serial 57600 8N1. Тот же UART, что и клавиатура
// PC-98, поэтому при DEBUG передача в машину выключена.
// В мониторе порта строки мыши: M <байты> x <dx> y <dy>
////////////////////////////////////////////////////////////////////////////////

//#define DEBUG

#define PHASE_X_US 150
#define PHASE_Y_US 175
#define MOUSE_MAX_STEP 20
#define PEND_MAX 60

#define pc98tm 0x22
#define numlock
#define numkeep

#define KQ_MASK 31
#define MQ_MASK 31

#define BUS_XA _BV(0)
#define BUS_XB _BV(1)
#define BUS_YA _BV(2)
#define BUS_YB _BV(3)
#define BUS_L  _BV(4)
#define BUS_ALL ((uint8_t)(BUS_XA | BUS_XB | BUS_YA | BUS_YB | BUS_L))
#define RBTN_LO() do { PORTD &= (uint8_t)~_BV(5); DDRD |= _BV(5); } while (0)
#define RBTN_HI() do { DDRD &= (uint8_t)~_BV(5); PORTD &= (uint8_t)~_BV(5); } while (0)

// PS/2 clock: PD2 мышь, PD3 клавиатура. Data: PC0 мышь, PC1 клавиатура.
#define KCLK_LO() do { PORTD &= (uint8_t)~_BV(3); DDRD |= _BV(3); } while (0)
#define KCLK_HI() do { DDRD &= (uint8_t)~_BV(3); PORTD |= _BV(3); } while (0)
#define KDAT_LO() do { PORTC &= (uint8_t)~_BV(1); DDRC |= _BV(1); } while (0)
#define KDAT_HI() do { DDRC &= (uint8_t)~_BV(1); PORTC |= _BV(1); } while (0)
#define KDAT_BIT() ((PINC >> 1) & 1)

#define MCLK_LO() do { PORTD &= (uint8_t)~_BV(2); DDRD |= _BV(2); } while (0)
#define MCLK_HI() do { DDRD &= (uint8_t)~_BV(2); PORTD |= _BV(2); } while (0)
#define MDAT_LO() do { PORTC &= (uint8_t)~_BV(0); DDRC |= _BV(0); } while (0)
#define MDAT_HI() do { DDRC &= (uint8_t)~_BV(0); PORTC |= _BV(0); } while (0)
#define MDAT_BIT() (PINC & 1)
#define MCLK_BIT() ((PIND >> 2) & 1)

#define RST_LO() ((PIND & _BV(6)) == 0)
#define RDY_HI() ((PIND & _BV(7)) != 0)
#define RTY_LO() ((PIND & _BV(4)) == 0)

volatile uint8_t ps2clk = 10;
volatile uint8_t ps2data = 0;
volatile uint8_t scancode = 0;
volatile uint8_t status = 0x04;
volatile uint8_t kbd_sending = 0;

static uint8_t locks = 0;
static uint8_t tmrate = pc98tm;
static uint8_t kbd_parity = 1;
static uint8_t macro[] = {0x21, 0x1D, 0x2F, 0x12, 0x2B, 0x2D, 0x1C};

static volatile uint8_t kq[32];
static volatile uint8_t kq_w = 0;
static volatile uint8_t kq_r = 0;
static volatile uint16_t kbd_edge = 0;

static volatile uint8_t mq[32];
static volatile uint8_t mq_w = 0;
static volatile uint8_t mq_r = 0;
static volatile uint8_t m_clk = 10;
static volatile uint8_t m_acc = 0;
static volatile uint16_t m_edge = 0;

static uint8_t mouse_ok = 0;
static uint8_t mouse_id = 0;
static uint8_t mouse_len = 3;
static int16_t pend_x = 0;
static int16_t pend_y = 0;
static int8_t st_x = 0;
static int8_t st_y = 0;
static uint32_t quad_next = 0;

void kq_push(uint8_t b) {
  uint8_t n = (uint8_t)((kq_w + 1) & KQ_MASK);
  if (n != kq_r) {
    kq[kq_w] = b;
    kq_w = n;
  }
}

void kq_flush(void) {
  uint8_t s = SREG;
  cli();
  kq_r = kq_w;
  SREG = s;
}

uint8_t kq_pop(uint8_t *b) {
  if (kq_r == kq_w) return 0;
  *b = kq[kq_r];
  kq_r = (uint8_t)((kq_r + 1) & KQ_MASK);
  return 1;
}

void mq_push(uint8_t b) {
  uint8_t n = (uint8_t)((mq_w + 1) & MQ_MASK);
  if (n != mq_r) {
    mq[mq_w] = b;
    mq_w = n;
  }
}

void bus_low(uint8_t m) { DDRB |= m; }
void bus_rel(uint8_t m) { DDRB &= (uint8_t)~m; }

void x_out(int8_t s) {
  switch (s & 3) {
    case 0: bus_low(BUS_XA); bus_rel(BUS_XB); break;
    case 1: bus_low(BUS_XA); bus_low(BUS_XB); break;
    case 2: bus_rel(BUS_XA); bus_low(BUS_XB); break;
    default: bus_rel(BUS_XA); bus_rel(BUS_XB); break;
  }
}

void y_out(int8_t s) {
  switch (s & 3) {
    case 3: bus_low(BUS_YA); bus_rel(BUS_YB); break;
    case 2: bus_low(BUS_YA); bus_low(BUS_YB); break;
    case 1: bus_rel(BUS_YA); bus_low(BUS_YB); break;
    default: bus_rel(BUS_YA); bus_rel(BUS_YB); break;
  }
}

void quad_step(void) {
  int8_t inc;
  uint32_t now;
  if (!pend_x && !pend_y) return;
  now = micros();
  if ((int32_t)(now - quad_next) < 0) return;
  if (pend_x) {
    inc = (pend_x > 0) ? 1 : -1;
    pend_x = (int16_t)(pend_x - inc);
    x_out(st_x);
    st_x = (int8_t)(st_x + inc);
    if (st_x < 0) st_x = 3;
    if (st_x > 3) st_x = 0;
    quad_next = now + PHASE_X_US;
  } else {
    inc = (pend_y > 0) ? 1 : -1;
    pend_y = (int16_t)(pend_y - inc);
    y_out(st_y);
    st_y = (int8_t)(st_y + inc);
    if (st_y < 0) st_y = 3;
    if (st_y > 3) st_y = 0;
    quad_next = now + PHASE_Y_US;
  }
}

void mouse_apply(const uint8_t *pkt) {
  int16_t dx;
  int16_t dy;
  if (pkt[0] & 0x01) bus_low(BUS_L); else bus_rel(BUS_L);
  if (pkt[0] & 0x02) RBTN_LO(); else RBTN_HI();
  dx = pkt[1];
  dy = pkt[2];
  if (pkt[0] & 0x10) dx |= (int16_t)0xFF00;
  if (pkt[0] & 0x20) dy |= (int16_t)0xFF00;
  if (dx > MOUSE_MAX_STEP) dx = MOUSE_MAX_STEP;
  if (dx < -MOUSE_MAX_STEP) dx = -MOUSE_MAX_STEP;
  if (dy > MOUSE_MAX_STEP) dy = MOUSE_MAX_STEP;
  if (dy < -MOUSE_MAX_STEP) dy = -MOUSE_MAX_STEP;
  pend_x = (int16_t)(pend_x + dx);
  pend_y = (int16_t)(pend_y + dy);
  if (pend_x > PEND_MAX) pend_x = PEND_MAX;
  if (pend_x < -PEND_MAX) pend_x = -PEND_MAX;
  if (pend_y > PEND_MAX) pend_y = PEND_MAX;
  if (pend_y < -PEND_MAX) pend_y = -PEND_MAX;
#ifdef DEBUG
  Serial.print(F("M"));
  for (uint8_t i = 0; i < mouse_len; i++) {
    Serial.print(' ');
    if (pkt[i] < 16) Serial.print('0');
    Serial.print(pkt[i], HEX);
  }
  Serial.print(F(" x "));
  Serial.print(dx);
  Serial.print(F(" y "));
  Serial.println(dy);
#endif
}

void mouse_parse(void) {
  static uint8_t pkt[4];
  static uint8_t pi = 0;
  uint8_t b;
  while (mq_r != mq_w) {
    b = mq[mq_r];
    mq_r = (uint8_t)((mq_r + 1) & MQ_MASK);
    if (pi == 0 && (b & 0x08) == 0) {
#ifdef DEBUG
      Serial.print(F("M skip "));
      Serial.println(b, HEX);
#endif
      continue;
    }
    pkt[pi++] = b;
    if (pi >= mouse_len) {
      mouse_apply(pkt);
      pi = 0;
    }
  }
}

void idle(uint16_t ms) {
  uint32_t t = millis();
  while ((uint16_t)(millis() - t) < ms) {
    mouse_parse();
    quad_step();
  }
}

// Поток PS/2: 11 спадов clock на байт. Первый после паузы ? старт, его не копим.
void mouse_isr(void) {
  uint8_t bit = MDAT_BIT();
  uint16_t now = (uint16_t)micros();
  uint8_t gap = (uint16_t)(now - m_edge) > 2000;
  m_edge = now;
  if (gap || m_clk > 9) {
    m_clk = 0;
    m_acc = 0;
    return;
  }
  if (m_clk < 8) {
    m_acc |= (uint8_t)((unsigned)bit << m_clk);
    if (m_clk == 7) mq_push(m_acc);
  }
  m_clk++;
}

void kbd_isr(void) {
  uint8_t bit;
  uint8_t outgoing;
  uint16_t now;
  uint8_t gap;
  if (kbd_sending) {
    switch (ps2clk) {
      case 0: case 1: case 2: case 3:
      case 4: case 5: case 6: case 7:
        outgoing = (ps2data >> ps2clk) & 1;
        if (outgoing) KDAT_HI(); else KDAT_LO();
        kbd_parity ^= outgoing;
        break;
      case 8:
        if (kbd_parity) KDAT_HI(); else KDAT_LO();
        break;
      case 9:
        KDAT_HI();
        break;
    }
    if (ps2clk > 10) {
      ps2clk = 0;
      kbd_parity = 1;
    } else {
      ps2clk++;
    }
    return;
  }
  bit = KDAT_BIT();
  now = (uint16_t)micros();
  gap = (uint16_t)(now - kbd_edge) > 2000;
  kbd_edge = now;
  if (gap || ps2clk > 9) {
    ps2clk = 0;
    ps2data = 0;
    return;
  }
  ps2data |= (uint8_t)((unsigned)bit << ps2clk);
  if (status & 0x10) {
    // Быстрые карты смотрят незаконченный байт. Коды в таблицах уже +1.
    scancode = (uint8_t)(ps2data + 1);
    ps2clk++;
    return;
  }
  if (ps2clk == 7) kq_push(ps2data);
  ps2clk++;
}

uint8_t mwait(uint8_t high, uint16_t ms) {
  uint32_t t = millis();
  while ((uint8_t)(MCLK_BIT() ? 1 : 0) != high) {
    if ((millis() - t) >= ms) return 0;
  }
  return 1;
}

void m_idle(void) {
  MDAT_HI();
  MCLK_HI();
}

uint8_t mouse_write(uint8_t data) {
  uint8_t parity = 1;
  uint8_t i;
  uint32_t t;
  m_idle();
  delayMicroseconds(300);
  MCLK_LO();
  delayMicroseconds(300);
  MDAT_LO();
  delayMicroseconds(10);
  MCLK_HI();
  t = millis();
  while (MCLK_BIT()) {
    if ((millis() - t) >= 100) {
      m_idle();
      return 0;
    }
  }
  for (i = 0; i < 8; i++) {
    if (data & 1) MDAT_HI(); else MDAT_LO();
    if (!mwait(1, 15) || !mwait(0, 15)) {
      m_idle();
      return 0;
    }
    parity ^= (uint8_t)(data & 1);
    data >>= 1;
  }
  if (parity) MDAT_HI(); else MDAT_LO();
  if (!mwait(1, 15) || !mwait(0, 15)) {
    m_idle();
    return 0;
  }
  MDAT_HI();
  t = millis();
  while (MDAT_BIT()) {
    if ((millis() - t) >= 25) {
      m_idle();
      return 0;
    }
  }
  while (MCLK_BIT()) {
    if ((millis() - t) >= 25) {
      m_idle();
      return 0;
    }
  }
  while (!MCLK_BIT() && !MDAT_BIT()) {
    if ((millis() - t) >= 25) {
      m_idle();
      return 0;
    }
  }
  return 1;
}

int mouse_read(void) {
  uint8_t data = 0;
  uint8_t i;
  uint32_t t = millis();
  m_idle();
  delayMicroseconds(50);
  while (MCLK_BIT()) {
    if ((millis() - t) >= 100) return -1;
  }
  while (!MCLK_BIT()) {
    if ((millis() - t) >= 100) return -1;
  }
  for (i = 0; i < 8; i++) {
    while (MCLK_BIT()) {
      if ((millis() - t) >= 100) return -1;
    }
    if (MDAT_BIT()) data |= (uint8_t)(1u << i);
    while (!MCLK_BIT()) {
      if ((millis() - t) >= 100) return -1;
    }
  }
  while (MCLK_BIT()) {
    if ((millis() - t) >= 100) return -1;
  }
  while (!MCLK_BIT()) {
    if ((millis() - t) >= 100) return -1;
  }
  while (MCLK_BIT()) {
    if ((millis() - t) >= 100) return -1;
  }
  while (!MCLK_BIT()) {
    if ((millis() - t) >= 100) return -1;
  }
  return data;
}

void mouse_rate(uint8_t rate) {
  int a;
  if (!mouse_write(0xF3)) return;
  a = mouse_read();
  if (a < 0) return;
  if (!mouse_write(rate)) return;
  (void)mouse_read();
}

uint8_t mouse_init(void) {
  int bat;
  int ack;
  int id;
  m_idle();
  if (!mouse_write(0xFF)) return 0;
  bat = mouse_read();
  if (bat < 0 || bat == 0xFC) return 0;
  (void)mouse_read();
  mouse_rate(200);
  mouse_rate(100);
  mouse_rate(80);
  mouse_id = 0;
  if (mouse_write(0xF2)) {
    ack = mouse_read();
    id = mouse_read();
    if (ack == 0xFA && id >= 0) mouse_id = (uint8_t)id;
  }
  if (!mouse_write(0xF4)) return 0;
  (void)mouse_read();
  mouse_len = (mouse_id == 0x03 || mouse_id == 0x04) ? 4 : 3;
  m_clk = 10;
  m_acc = 0;
  mq_r = mq_w = 0;
  return 1;
}

void ports_init(void) {
  PORTB &= (uint8_t)~(BUS_ALL | _BV(5));
  DDRB &= (uint8_t)~(BUS_ALL | _BV(5));
  DDRD &= (uint8_t)~_BV(5);
  PORTD &= (uint8_t)~_BV(5);
  // ~RST ~RDY ~RTY: вход с подтяжкой, активный уровень низкий, его даёт хост.
  DDRD &= (uint8_t)~(_BV(4) | _BV(6) | _BV(7));
  PORTD |= (uint8_t)(_BV(4) | _BV(6) | _BV(7));
  KCLK_HI();
  KDAT_HI();
  MCLK_HI();
  MDAT_HI();
}

void ps2send(uint8_t command) {
  uint32_t exectime;
  uint32_t t0 = millis();
  for (;;) {
    mouse_parse();
    quad_step();
    uint8_t sreg = SREG;
    cli();
    if (ps2clk >= 10 || (millis() - t0) > 20) {
      detachInterrupt(digitalPinToInterrupt(3));
      ps2data = command;
      ps2clk = 12;
      kbd_sending = 1;
      SREG = sreg;
      break;
    }
    SREG = sreg;
  }
  idle(5);
  exectime = millis();
  KCLK_LO();
  delayMicroseconds(100);
  KDAT_LO();
  attachInterrupt(digitalPinToInterrupt(3), kbd_isr, FALLING);
  KCLK_HI();
  while ((millis() - exectime) < 19) {
    if (ps2clk == 11) break;
    mouse_parse();
    quad_step();
  }
  if (ps2clk != 11) KDAT_HI();
  detachInterrupt(digitalPinToInterrupt(3));
  kbd_sending = 0;
  ps2clk = 10;
  ps2data = 0;
  attachInterrupt(digitalPinToInterrupt(3), kbd_isr, FALLING);
}

void pc98send(uint8_t sendcode) {
  static uint8_t lastcode = 0;
  if (sendcode < 0x80) {
    if (status & 0x01) {
      sendcode |= 0x80;
      status &= (uint8_t)~0x01;
    }
#ifdef DEBUG
    Serial.print(F("K out "));
    Serial.println(sendcode, HEX);
#else
    if (RTY_LO()) {
      while (RDY_HI()) {
        mouse_parse();
        quad_step();
      }
      Serial.write(lastcode);
    }
    if (RST_LO()) {
      while (RST_LO()) {
        mouse_parse();
        quad_step();
      }
      idle(13);
      while (RDY_HI()) {
        mouse_parse();
        quad_step();
      }
      delayMicroseconds(100);
    } else {
      while (RDY_HI()) {
        mouse_parse();
        quad_step();
      }
    }
    Serial.write(sendcode);
#endif
    lastcode = sendcode;
  }
  if (sendcode == 0xFF) status &= (uint8_t)~0x01;
  scancode = 0;
}

void kbd_reset(void) {
  ps2send(0xFF);
  idle(750);
  ps2send(0xF3);
  ps2send(tmrate);
#ifndef numkeep
#ifdef numlock
  status &= (uint8_t)~0x04;
  locks |= 0x02;
#else
  status |= 0x04;
  locks &= (uint8_t)~0x02;
#endif
#endif
}

void tmset(void) {
  ps2send(0xF3);
  ps2send(tmrate);
}

void ledset(uint8_t ledstatus) {
  ps2send(0xED);
  if (ledstatus < 0x10) ps2send(ledstatus);
  else ps2send(locks & 0x0F);
}

// Ждёт ответы на команду и возвращает последний байт (id набора, не ACK 0xFA).
uint8_t kbd_last_byte(uint8_t window_ms) {
  uint8_t b = 0;
  uint8_t got = 0;
  uint8_t v;
  uint32_t t0 = millis();
  uint32_t last = t0;
  while ((millis() - t0) < window_ms) {
    mouse_parse();
    quad_step();
    if (kq_pop(&v)) {
      b = v;
      got = 1;
      last = millis();
    } else if (got && (millis() - last) > 8) {
      break;
    }
  }
  return got ? b : 0;
}

uint8_t codeset(uint8_t codeid) {
  if (codeid > 0) {
    ps2send(0xF0);
    ps2send(codeid);
    (void)kbd_last_byte(20);
  }
  ps2send(0xF0);
  ps2send(0x00);
  return kbd_last_byte(25);
}

void locktgl(uint8_t lockid) {
  if (status & 0x01) {
    scancode = 0xFF;
    locks &= (uint8_t)~(0x10 << lockid);
  } else {
    if ((locks >> lockid) & 0x10) scancode = 0xFF;
    else {
      switch (lockid) {
        case 0: scancode = 0x72; break;
        case 1: scancode = 0xFF; status ^= 0x04; break;
        case 2: scancode = 0x71; break;
      }
      if ((locks >> lockid) & 0x01) status |= 0x01;
      locks ^= (uint8_t)(1u << lockid);
      locks |= (uint8_t)(0x10 << lockid);
    }
  }
  ledset(0x10);
}

void nextmap(void) {
  int8_t map;
  detachInterrupt(digitalPinToInterrupt(3));
  idle(50);
  if (scancode == 0x7E) {
    status |= 0x01;
    scancode = 0x74;
    pc98send(scancode);
    idle(20);
    status |= 0x01;
    scancode = 0x33;
    pc98send(scancode);
  }
  kq_flush();
  ps2clk = 10;
  ps2data = 0;
  map = (int8_t)(status >> 5);
  status &= 0x0C;
  if (map < 3) map++; else map = 0;
  status |= (uint8_t)(map << 5);
  if (map == 1) {
    if (scancode != 0 && codeset(0x03) == 0x03) {
      status |= 0x08;
      ps2send(0xF8);
    }
    status |= 0x10;
  }
  if (map > 1 && (status & 0x08)) {
    if (codeset(0x02) == 0x02) status &= (uint8_t)~0x08;
    else {
      map = 0;
      status &= 0x04;
      ledset(0x01); idle(100);
      ledset(0x04); idle(100);
      ledset(0x02); idle(100);
      ledset(0x04); idle(100);
      ledset(0x01); idle(100);
      ledset(0x00); idle(100);
      ledset(0x01); idle(100);
      ledset(0x00); idle(100);
      ledset(0x01); idle(100);
      ledset(0x00); idle(100);
      ledset(0x01); idle(500);
      ledset(0x00); idle(500);
    }
  }
  if (map > 0) {
    tmrate = 0x7F;
    tmset();
    while (map > -1) {
      ledset(0x00); idle(100);
      ledset(0x07);
      if (map > 2) { idle(200); map = (int8_t)(map - 3); }
      else { idle(100); map--; }
    }
    ledset(0x00);
    if (status & 0x08) {
      idle(100);
      ledset(0x01); idle(500);
      ledset(0x00);
    }
  } else {
    tmrate = pc98tm;
    kbd_reset();
    ledset(0x10);
  }
  if (status & 0x10) idle(50);
  kbd_sending = 0;
  ps2clk = 10;
  attachInterrupt(digitalPinToInterrupt(3), kbd_isr, FALLING);
}

void convfull() {  //Full standard 101/102-key layout conversion, unusual mappings are marked with arrows
  switch (scancode) {                   //PS/2 key (PC-98 KEY, if different)
    case 0x01: scancode = 0x6A; break;  //F9
    case 0x03: scancode = 0x66; break;  //F5
    case 0x04: scancode = 0x64; break;  //F3
    case 0x05: scancode = 0x62; break;  //F1
    case 0x06: scancode = 0x63; break;  //F2
    case 0x07: scancode = 0x53; break;  //F12 (VF2)
    //case 0x08: scancode = 0x54; break;  //F13 (VF3)
    case 0x09: scancode = 0x6B; break;  //F10
    case 0x0A: scancode = 0x69; break;  //F8
    case 0x0B: scancode = 0x67; break;  //F6
    case 0x0C: scancode = 0x65; break;  //F4
    case 0x0D: scancode = 0x0F; break;  //Tab
    case 0x0E: scancode = 0x1A; break;  //~ (@) <-------------------
    //case 0x10: scancode = 0x55; break;  //F14 (VF4)
    case 0x11: scancode = 0x73; status &= 0b11111101; break;  //LAlt, RAlt (GRPH)
    case 0x12: if (status & 0b00000010) {scancode = 0xFF; status &= 0b11111101;} else scancode = 0x70; break;  //LShift (SHIFT), dismiss fake shifts
    case 0x14: if (status & 0b00000010) {scancode = 0x33; status &= 0b11111101;} else scancode = 0x74; break;  //LCtrl (CTRL), RCtrl (--) <-------------------
    case 0x15: scancode = 0x10; break;  //Q
    case 0x16: scancode = 0x01; break;  //1
    //case 0x18: scancode = 0x56; break;  //F15 (VF5)
    case 0x1A: scancode = 0x29; break;  //Z
    case 0x1B: scancode = 0x1E; break;  //S
    case 0x1C: scancode = 0x1D; break;  //A
    case 0x1D: scancode = 0x11; break;  //W
    case 0x1E: scancode = 0x02; break;  //2
    case 0x1F: scancode = 0x51; status &= 0b11111101; break;  //LWin (NFER) <-------------------
    case 0x21: scancode = 0x2B; break;  //C
    case 0x22: scancode = 0x2A; break;  //X
    case 0x23: scancode = 0x1F; break;  //D
    case 0x24: scancode = 0x12; break;  //E
    case 0x25: scancode = 0x04; break;  //4
    case 0x26: scancode = 0x03; break;  //3
    case 0x27: scancode = 0x35; status &= 0b11111101; break;  //RWin (XFER) <-------------------
    case 0x29: scancode = 0x34; break;  //Space
    case 0x2A: scancode = 0x2C; break;  //V
    case 0x2B: scancode = 0x20; break;  //F
    case 0x2C: scancode = 0x14; break;  //T
    case 0x2D: scancode = 0x13; break;  //R
    case 0x2E: scancode = 0x05; break;  //5
    case 0x2F: locktgl(0); status &= 0b11111101; break;  //Menu (KANA) <-------------------
    case 0x31: scancode = 0x2E; break;  //N
    case 0x32: scancode = 0x2D; break;  //B
    case 0x33: scancode = 0x22; break;  //H
    case 0x34: scancode = 0x21; break;  //G
    case 0x35: scancode = 0x15; break;  //Y
    case 0x36: scancode = 0x06; break;  //6
    case 0x37: scancode = 0x54; status &= 0b11111101; break;  //Power (VF3) <-------------------
    case 0x3A: scancode = 0x2F; break;  //M
    case 0x3B: scancode = 0x23; break;  //J
    case 0x3C: scancode = 0x16; break;  //U
    case 0x3D: scancode = 0x07; break;  //7
    case 0x3E: scancode = 0x08; break;  //8
    case 0x3F: scancode = 0x55; status &= 0b11111101; break;  //Sleep (VF4) <-------------------
    case 0x41: scancode = 0x30; break;  //,
    case 0x42: scancode = 0x24; break;  //K
    case 0x43: scancode = 0x17; break;  //I
    case 0x44: scancode = 0x18; break;  //O
    case 0x45: scancode = 0x0A; break;  //0
    case 0x46: scancode = 0x09; break;  //9
    case 0x49: scancode = 0x31; break;  //.
    case 0x4A: if (status & 0b00000010) {scancode = 0x41; status &= 0b11111101;} else scancode = 0x32; break;  // /, Numpad /
    case 0x4B: scancode = 0x25; break;  //L
    case 0x4C: scancode = 0x26; break;  //: (;)
    case 0x4D: scancode = 0x19; break;  //P
    case 0x4E: scancode = 0x0B; break;  //-
    case 0x52: scancode = 0x27; break;  //" (:)
    case 0x54: scancode = 0x1B; break;  //[ <-------------------
    case 0x55: scancode = 0x0C; break;  //= (^)
    case 0x58: locktgl(2); break;  //Caps Lock
    case 0x59: if (status & 0b00000010) {scancode = 0xFF; status &= 0b11111101;} else scancode = 0x70; break;  //RShift (SHIFT), dismiss fake shifts
    case 0x5A: if (status & 0b00000010) {scancode = 0x4D; status &= 0b11111101;} else scancode = 0x1C; break;  //Enter, Numpad Enter (NUMPAD =) <-------------------
    case 0x5B: scancode = 0x28; break;  //] <-------------------
    case 0x5D: scancode = 0x0D; break;  //Backslash (YEN)
    case 0x5E: scancode = 0x56; status &= 0b11111101; break;  //Wake (VF5) <-------------------
    case 0x61: scancode = 0x0D; break;  //ISO Backslash (YEN)
    //case 0x61: scancode = 0x70; break;  //ISO Backslash (SHIFT)
    case 0x66: scancode = 0x0E; break;  //Backspace
    case 0x69: if (status & 0b00000110) {scancode = 0x3F; status &= 0b11111101;} else scancode = 0x4A; break;  //Numpad 1, End (HELP)
    case 0x6B: if (status & 0b00000110) {scancode = 0x3B; status &= 0b11111101;} else scancode = 0x46; break;  //Numpad 4, Left
    case 0x6C: if (status & 0b00000110) {scancode = 0x3E; status &= 0b11111101;} else scancode = 0x42; break;  //Numpad 7, Home (HOME CLR)
    //case 0x6C: if (status & 0b00000110) {scancode = 0x5E; status &= 0b11111101;} else scancode = 0x42; break;  //Numpad 7, Home (HOME)
    case 0x70: if (status & 0b00000110) {scancode = 0x38; status &= 0b11111101;} else scancode = 0x4E; break;  //Numpad 0, Insert
    //case 0x71: if (status & 0b00000110) {scancode = 0x39; status &= 0b11111101;} else scancode = 0x4F; break;  //Numpad . (,), Delete
    case 0x71: if (status & 0b00000110) {scancode = 0x39; status &= 0b11111101;} else scancode = 0x50; break;  //Numpad . (.), Delete
    case 0x72: if (status & 0b00000110) {scancode = 0x3D; status &= 0b11111101;} else scancode = 0x4B; break;  //Numpad 2, Down
    case 0x73: if (status & 0b00000100) scancode = 0xFF; else scancode = 0x47; break;  //Numpad 5
    case 0x74: if (status & 0b00000110) {scancode = 0x3C; status &= 0b11111101;} else scancode = 0x48; break;  //Numpad 6, Right
    case 0x75: if (status & 0b00000110) {scancode = 0x3A; status &= 0b11111101;} else scancode = 0x43; break;  //Numpad 8, Up
    case 0x76: scancode = 0x00; break;  //Esc
    case 0x77: locktgl(1); break;  //Num Lock
    case 0x78: scancode = 0x52; break;  //F11 (VF1)
    case 0x79: scancode = 0x49; break;  //Numpad +
    case 0x7A: if (status & 0b00000110) {scancode = 0x36; status &= 0b11111101;} else scancode = 0x4C; break;  //Numpad 3, Page Down (ROLL UP)
    case 0x7B: scancode = 0x40; break;  //Numpad -
    case 0x7C: if (status & 0b00000010) {scancode = 0x60; status &= 0b11111101;} else scancode = 0x45; break;  //Numpad *, Print Screen (STOP) <-------------------
    case 0x7D: if (status & 0b00000110) {scancode = 0x37; status &= 0b11111101;} else scancode = 0x44; break;  //Numpad 9, Page Up (ROLL DOWN)
    case 0x7E: if (status & 0b00000010) nextmap(); else scancode = 0x61; break;  //Scroll Lock (COPY), Ctrl + Pause Break (switch to next map) <-------------------
    case 0x83: scancode = 0x68; break;  //F7
    case 0x84: if (!status & 0b00000001) for (uint8_t i = 0; i < sizeof(macro); i++) {
        pc98send(macro[i]);  //Send macro make scancode
        idle(20);  //This delay is an approximation, increase if something goes wrong
        status |= 0b00000001;  //Signal key up
        pc98send(macro[i]);  //Send macro break scancode
        idle(20);  //This delay is an approximation, increase if something goes wrong
      } scancode = 0xFF; break;  //Alt + Print Screen (macro)
    case 0xE0: status |= 0b00000010; break;  //Set extend flag
    case 0xE1: nextmap(); break;  //Pause Break (switch to next map)
    case 0xF0: status |= 0b00000001; break;  //Set keybreak
    default: scancode = 0xFF;  //Invalid scancode (do not output)
  }
  pc98send(scancode);
}

void convtoho() {  //Fast predictive converter for a certain game series, lots of phantom mappings (see if you can find an unintended feature)
  //Can't be easily debugged like other converters due to tight timings
  switch (scancode) {  //PS2 key (PC98 KEY, if different)
    case 0x03: if (ps2clk == 5) scancode = 0x2A; else scancode = 0xFE; break;  //X
    case 0x04: if (ps2clk == 3) scancode = 0x3B; else scancode = 0xFE; status &= 0b11111101; break;  //Left
    case 0x05: if (ps2clk == 3) scancode = 0x3C; else scancode = 0xFE; status &= 0b11111101; break; break;  //Right
    case 0x0A: if (ps2clk == 5) scancode = 0x34; else scancode = 0xFE; break;  //Space
    case 0x0B: scancode = 0x29; status &= 0b11111101; break;  //Z, Enter, Numpad Enter (Z)
    //case 0x0B: if (ps2clk == 4) scancode = 0x29; else scancode = 0xFE; status &= 0b11111101; break;  //Z, Enter, Numpad Enter
    case 0x13: if (ps2clk == 6) {if (status & 0b00000010) {scancode = 0xFF; status &= 0b11111101;} else scancode = 0x70;} else scancode = 0xFE; break;  //LShift (SHIFT), dismiss fake shifts
    case 0x16: if (ps2clk == 8) scancode = 0x10; else scancode = 0xFE; break;  //Q
    case 0x1A: scancode = 0x70; break;  //RShift (SHIFT)
    //case 0x1A: if (ps2clk == 5) scancode = 0x70; else scancode = 0xFE; break;  //RShift (SHIFT)
    case 0x33: scancode = 0x3D; status &= 0b11111101; break;  //Down, will stick if you press B, press and release down to fix, or use the line below (theoretically slower)
    //case 0x33: if (ps2clk == 6) scancode = 0x3D; else scancode = 0xFE; status &= 0b11111101; break;  //Down
    case 0x36: scancode = 0x3A; status &= 0b11111101; break;  //Up, will stick if you press Y, press and release down to fix, or use the line below (theoretically slower)
    //case 0x36: if (ps2clk == 6) scancode = 0x3A; else scancode = 0xFE; status &= 0b11111101; break;  //Up
    case 0x62: if (ps2clk == 8) scancode = 0x70; else scancode = 0xFE; break;  //ISO Backslash (SHIFT)
    case 0x77: if (ps2clk == 8) scancode = 0x00; else scancode = 0xFE; break;  //Esc
    //case 0x78: locktgl(1); break;  //Num Lock, indication will be broken, see locktgl
    //PRO TIP: press Num Lock odd number of times (once) after switching the map to reduce the amount of ignored scancodes (1 vs 4) generated when arrow keys are pressed with shift down.
    //         This, however, produces more scancodes for unshifted presses (3 vs 1), so not suitable for Reiiden, Fuumaroku, Yumejikuu... Depends on the LED status in normal operation.
    case 0xE1: status |= 0b00000010; break;  //Set extend flag
    case 0xE2: nextmap(); break;  //Pause Break (switch to next map)
    case 0xF1: status |= 0b00000001; break;  //Set keybreak
    default: if (ps2clk == 8) scancode = 0xFF; else scancode = 0xFE;  //Invalid scancode (do not output)
  }
  pc98send(scancode);
}

void convgame() {  //Simple uniform 5-bit scan code set 3 predictive converter for games, should be faster overall (no need to wait for extends and fake shifts!!!)
  if (ps2clk == 5) {  //Fast 5-bit mappings
    switch (scancode) {                   //PS2 key (PC98 KEY, if different)
      case 0x01: scancode = 0x3D; break;  //Down
      case 0x02: scancode = 0x3B; break;  //C, , , Left (LEFT)
      case 0x03: scancode = 0x2A; break;  //X, K, Pause (X)
      case 0x04: scancode = 0x3A; break;  //D, I, Up (UP)
      case 0x05: scancode = 0x12; break;  //E, O, Delete (E)
      case 0x09: scancode = 0x00; break;  //Esc
      case 0x0A: scancode = 0x34; break;  //Space, . , Numpad 1 (SPACE)
      case 0x0B: scancode = 0x3C; break;  //V, /, Right (RIGHT)
      case 0x0C: scancode = 0x20; break;  //F, L, Numpad 4, LWin (F)
      case 0x0D: scancode = 0x14; break;  //T, ; , Numpad 7, RWin (T)
      case 0x12: scancode = 0x74; break;  //LCtrl, N, Numpad . (CTRL)
      case 0x13: scancode = 0x70; break;  //LShift, B, ', Numpad 2 (SHIFT)
      case 0x14: scancode = 0x70; break;  //H, ISO Backslash, Numpad 5 (SHIFT)
      case 0x15: scancode = 0x21; break;  //Caps Lock, G, [, Numpad 6 (G)
      case 0x16: scancode = 0x10; break;  //Q, Y, =, Numpad 8 (Q)
      case 0x19: scancode = 0x74; break;  //RCtrl (CTRL)
      case 0x1A: scancode = 0x70; break;  //LAlt, RAlt, RShift, Numpad Enter (SHIFT)
      case 0x1B: scancode = 0x29; break;  //Z, M, Enter, Numpad 3 (Z)
      case 0x1C: scancode = 0x1E; break;  //S, J, ] (S)
      case 0x1D: break;                   //A, U, Backslash, Numpad + (A)
      default: scancode = 0xFE;  //Wait to receive the rest of the scancode
    }
    pc98send(scancode);
  } else if (ps2clk == 8) {  //Slow 8-bit mappings, for occasions
    switch (scancode) {                   //PS2 key (PC98 KEY, if different)
      case 0x08: scancode = 0x62; break;  //F1
      case 0x0E: scancode = 0x0F; break;  //Tab
      case 0x0F: scancode = 0x1A; break;  //~ (@)
      case 0x10: scancode = 0x63; break;  //F2
      case 0x17: scancode = 0x01; break;  //1
      case 0x18: scancode = 0x64; break;  //F3
      case 0x1E: scancode = 0x11; break;  //W
      case 0x1F: scancode = 0x02; break;  //2
      case 0x20: scancode = 0x65; break;  //F4
      case 0x26: scancode = 0x04; break;  //4
      case 0x27: scancode = 0x03; break;  //3
      case 0x28: scancode = 0x66; break;  //F5
      case 0x2E: scancode = 0x13; break;  //R
      case 0x2F: scancode = 0x05; break;  //5
      case 0x30: scancode = 0x67; break;  //F6
      case 0x37: scancode = 0x06; break;  //6
      case 0x38: scancode = 0x68; break;  //F7
      case 0x3E: scancode = 0x07; break;  //7
      case 0x3F: scancode = 0x08; break;  //8
      case 0x40: scancode = 0x69; break;  //F8
      case 0x46: scancode = 0x0A; break;  //0
      case 0x47: scancode = 0x09; break;  //9
      case 0x48: scancode = 0x6A; break;  //F9
      case 0x4E: scancode = 0x19; break;  //P
      case 0x4F: scancode = 0x0B; break;  //-
      case 0x50: scancode = 0x6B; break;  //F10
      case 0x57: scancode = 0x52; break;  //F11 (VF1)
      case 0x58: scancode = 0x60; break;  //Print Screen (STOP)
      case 0x5F: scancode = 0x53; break;  //F12 (VF2)
      case 0x60: nextmap(); break;        //Scroll Lock (switch to next map)
      case 0x66: scancode = 0x3F; break;  //End (HELP)
      case 0x67: scancode = 0x0E; break;  //Backspace
      case 0x68: scancode = 0x38; break;  //Insert
      case 0x6E: scancode = 0x36; break;  //Page Down (ROLL UP)
      case 0x6F: scancode = 0x3E; break;  //Home (HOME CLR)
      case 0x70: scancode = 0x37; break;  //Page Up (ROLL DOWN)
      case 0x71: scancode = 0x1C; break;  //Numpad 0 (ENTER)
      case 0x77: scancode = 0x61; break;  //Num Lock (COPY)
      case 0x78: scancode = 0x0C; break;  //Numpad / (^)
      case 0x7E: scancode = 0x39; break;  //Numpad 9 (DEL)
      case 0x7F: scancode = 0x0D; break;  //Numpad * (YEN)
      case 0x8E: scancode = 0x73; break;  //Menu (GRPH)
      case 0xF1: status |= 0b00000001; break;  //Set keybreak
      default: scancode = 0xFF;  //Invalid scancode (do not output)
    }
    pc98send(scancode);
  } else scancode = 0;  //Ignore the rest of the bits
}


void convyume() {  //Comfortable layout for multiplayer in Yumejikuu, doesn't parse extend scancodes at all
  switch (scancode) {                   //PS2 key -------------- (In-game function)
    case 0x14: scancode = 0x3C; break;  //LCtrl, RCtrl --------- (P2 Bomb)
    case 0x15: scancode = 0x10; break;  //Q -------------------- (Quit)
    case 0x1A: scancode = 0x29; break;  //Z -------------------- (P1 Shot)
    case 0x22: scancode = 0x2A; break;  //X -------------------- (P1 Bomb)
    case 0x2F: scancode = 0x3B; break;  //Menu ----------------- (P2 Shot)
    case 0x33: scancode = 0x20; break;  //H -------------------- (P1 Left)
    case 0x3B: scancode = 0x2D; break;  //J -------------------- (P1 Down)
    case 0x3C: scancode = 0x14; break;  //U -------------------- (P1 Up)
    case 0x42: scancode = 0x22; break;  //K -------------------- (P1 Right)
    case 0x69: scancode = 0x46; break;  //Numpad 1, End -------- (P2 Left)
    case 0x72: scancode = 0x4B; break;  //Numpad 2, Down ------- (P2 Down)
    case 0x73: scancode = 0x43; break;  //Numpad 5 ------------- (P2 Up)
    case 0x76: scancode = 0x00; break;  //Esc ------------------ (Pause)
    //case 0x77: locktgl(1); break;  //Num Lock, indication will be broken, see locktgl
    case 0x7A: scancode = 0x48; break;  //Numpad 3, Page Down -- (P2 Right)
    case 0xE1: nextmap(); break;  //Pause Break (switch to next map)
    case 0xF0: status |= 0b00000001; break;  //Set keybreak
    default: scancode = 0xFF;  //Invalid scancode (do not output)
  }
  pc98send(scancode);
}

void convymsp() {  //Play against yourself in Yumejikuu, if you dare
  switch (scancode) {                   //PS2 key (In-game function)
    case 0x12: if (status & 0b00000010) {scancode = 0xFF; status &= 0b11111101;} else scancode = 0x2A; break;  //LShift (P1 Bomb), dismiss fake shifts
    case 0x15: scancode = 0x10; break;  //Q (Quit)
    case 0x1B: scancode = 0x2D; break;  //S (P1 Down)
    case 0x1C: scancode = 0x20; break;  //A (P1 Left)
    case 0x1D: scancode = 0x14; break;  //W (P1 Up)
    case 0x23: scancode = 0x22; break;  //D (P1 Right)
    case 0x29: scancode = 0x29; break;  //Space (P1 Shot)
    case 0x5A: scancode = 0x3C; status &= 0b11111101; break;  //Enter, Numpad Enter (P2 Bomb)
    case 0x6B: scancode = 0x46; status &= 0b11111101; break;  //Numpad 4 (P2 Left)
    case 0x70: scancode = 0x3B; status &= 0b11111101; break;  //Numpad 0 (P2 Shot)
    case 0x73: scancode = 0x4B; break;  //Numpad 5 (P2 Down)
    case 0x74: if (status & 0b00000010) {scancode = 0x3B; status &= 0b11111101;} else scancode = 0x48; break;  //Numpad 6 (P2 Right), Left (P2 Shot)
    case 0x75: scancode = 0x43; status &= 0b11111101; break;  //Numpad 8 (P2 Up)
    case 0x76: scancode = 0x00; break;  //Esc (Pause)
    //case 0x77: locktgl(1); break;  //Num Lock, indication will be broken, see locktgl
    case 0xE0: status |= 0b00000010; break;  //Set extend flag
    case 0xE1: nextmap(); break;  //Pause Break (switch to next map)
    case 0xF0: status |= 0b00000001; break;  //Set keybreak
    default: scancode = 0xFF;  //Invalid scancode (do not output)
  }
  pc98send(scancode);
}


void kbd_poll(void) {
  uint8_t map;
  uint8_t b;
  if (status & 0x10) {
    if (!scancode) return;
  } else {
    if (!kq_pop(&b)) return;
    scancode = b;
#ifdef DEBUG
    Serial.print(F("K in "));
    Serial.println(b, HEX);
#endif
  }
  map = (uint8_t)(status >> 5);
  if (map == 0) convfull();
  else if (map == 1) {
    if (status & 0x08) convgame();
    else convtoho();
  } else if (map == 2) convyume();
  else convymsp();
}

void setup(void) {
  uint8_t try_n;
  ports_init();
#ifdef DEBUG
  Serial.begin(57600);
  Serial.println(F("PC98dualPS2"));
#else
  Serial.begin(19200, SERIAL_8O1);
#endif
  delay(1000);
  mouse_ok = 0;
  for (try_n = 0; try_n < 2 && !mouse_ok; try_n++) mouse_ok = mouse_init();
#ifdef DEBUG
  if (!mouse_ok) Serial.println(F("mouse: no"));
  else {
    Serial.print(F("mouse id "));
    Serial.print(mouse_id, HEX);
    Serial.print(F(" len "));
    Serial.println(mouse_len);
  }
#endif
  if (mouse_ok) attachInterrupt(digitalPinToInterrupt(2), mouse_isr, FALLING);
  tmset();
#ifdef numlock
  status &= (uint8_t)~0x04;
  locks |= 0x02;
  ledset(0x10);
#endif
  kq_flush();
  scancode = 0;
}

void loop(void) {
  mouse_parse();
  quad_step();
  kbd_poll();
}
