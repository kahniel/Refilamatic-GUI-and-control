#include <LiquidCrystal_I2C.h> 
#include <EncButton.h>
#include <EEPROM.h>


// Именуем пины на плате
#define ENCBUT 3
#define ENCR 4
#define ENCL 5
#define BUTL 12
#define BUTR 13
#define DT A0
#define SCK A1
#define PUMP A2


LiquidCrystal_I2C lcd(0x27, 16, 2);
EncButton<EB_TICK, ENCL, ENCR, ENCBUT> enc;


// Спецсимволы для экрана
const int spnr = 0;
byte spnr_[] = {
  B00100,
  B01010,
  B10001,
  B00000,
  B11111,
  B01110,
  B00100,
  B00000
};
const int branch = 1;
byte branch_[] = {
  B00000,
  B00000,
  B00000,
  B00111,
  B11101,
  B00111,
  B00000,
  B00000
};
const int okBut = 2;
byte okBut_[] = {
  B00000,
  B00000,
  B00001,
  B00010,
  B10100,
  B01000,
  B00000,
  B11111
};
const int backBut = 3;
byte backBut_[] = {
  B00000,
  B00100,
  B01000,
  B11111,
  B01000,
  B00100,
  B00000,
  B11111
};
const int spnRight = 4;
byte spnRight_[] = {
  B00000,
  B00000,
  B01000,
  B01100,
  B01110,
  B01100,
  B01000,
  B00000
};
const int spnLeft = 5;
byte spnLeft_[] = {
  B00000,
  B00000,
  B00010,
  B00100,
  B01000,
  B00100,
  B00010,
  B00000
};
const int celc = 6;
byte celc_[] = {
  B11000,
  B11000,
  B00000,
  B01110,
  B10001,
  B10000,
  B10001,
  B01110
};
const int uno = 7;
byte customChar[] = {
  B00000,
  B01000,
  B11000,
  B01000,
  B01000,
  B01011,
  B01011,
  B00000
};


// Переменные работы с режимами
int mods[17][2]; // массив с данными режимов
// mods[0][1] – изменяемая в реальном времени температура создаваемого режима
// mods[1..7][1] – данные о температуре предустановленных режимов
// mods[8..16][0] == -1 – пользовательский режим с таким номером не установлен
// mods[8..16][1] – данные о температуре пользовательских режимов
int modq; // количество сохраненных режимов
int diam; // временно установленный диаметр для выбранного режима

// Переменные работы с меню
char state; // текущее состояние меню
char mod; // выбранный в меню режим
char pnt; // номер выделенного фрагмента второй строки экрана
char l, r; // левая и правая граница выделенного фрагмента
bool txtOn; // мигание выделенного фрагмента
char scrmem[16]; // текст второй строки экрана
int chDelChar = 1; // выбранный параметр удаляемого режима

// Кнопки лева и права
bool butlSt, butrSt;

// Процедуры аппаратной части



// Функции и процедуры работы с режимами

// Управление хранением mods в постоянной памяти
void modsSetUp() {
  for (int i = 1; i <= 7; ++i) mods[i][0] = 1;
  mods[1][1] = 190; // PLA
  mods[2][1] = 210; // PC
  mods[3][1] = 320; // PP
  mods[4][1] = 300; // PETG
  mods[5][1] = 180; // Neylon
  mods[6][1] = 280; // ABS
  mods[7][1] = 240; // ABS + PC
}

void modsBackUp() {
  EEPROM[0] = modq;
  int p = 1;
  for (int i = 8; i <= 16; ++i) {
    if (mods[i][0] == -1) {
      EEPROM[p] = 0;
      ++p;
    } else {
      EEPROM[p] = (mods[i][1] > 200) + 1;
      ++p;
      EEPROM[p] = (mods[i][1] > 200) ? (mods[i][1] - 200) : (mods[i][1]);
      ++p;
    }
  }
}
// Записать данных из постоянной памяти в mods
void modsFill() {
  modq = EEPROM[0];
  int p = 1;
  for (int i = 8; i <= 16; ++i) {
    if (EEPROM[p] == 0) {
      mods[i][0] = -1;
      ++p;
    } else {
      mods[i][0] = 1;
      mods[i][1] = (int)(EEPROM[p] - 1) * 200 + (int)(EEPROM[p + 1]);
      ++p;
      ++p;
    }
  }
}
// Очистить режимы
void modsClean() {
  modq = 0;
  for (int i = 8; i <= 16; ++i) mods[i][0] = -1;
  modsBackUp();
}

// Удалить режим m
void delMod(int m) {
  --modq;
  mods[m][0] = -1;
}

// Вернуть наименьший номер сохраненного режима
int firstMod() {
  int i = 1;
  while (mods[i][0] == -1) ++i;
  return i;
}
// Вернуть наименьший номер несохраненного режима
int firstFree() {
  int i = 1;
  while (mods[i][0] != -1) ++i;
  return i;
}
// Вернуть наибольший номер сохраненного режима
int lastMod() {
  int i = 16;
  while (mods[i][0] == -1) --i;
  return i;
}
// Перейти в направлении dir к ближайшему сохраненному режиму
int setMod(int dir, bool edble = false) {
  mod += dir;
  while (mods[mod][0] == -1) mod += dir;
  if (edble) {
    while (mod < 8 and mods[mod][0] == -1) mod += dir;
  }
  return mod;
}


// Функции и процедуры интерфейса

// Проверить, может ли быть выделен фрагмент pnt
bool badPnt() {
  if (state == 'm') return false;
  if (state == 'n') return false;
  if (state == 'd') return (pnt == 1 && modq == 1);
  if (state == 's') return false;
  if (state == 'r') return false;
}

// Перезаписать вторую строку на экран
void rewrite() {
  if (txtOn != 1 && 0 < millis() % 250 && millis() % 250 < 100) txtOn = 0;
  if (millis() % 250)
  lcd.setCursor(0, 1);
  for (int i = 0; i < 16; ++i) {
    if (!(l <= i && i <= r && txtOn == 0)) {
      if (scrmem[i] == '^') lcd.write(byte(spnr));
      else if (scrmem[i] == '<') lcd.write(byte(spnLeft));
      else if (scrmem[i] == '>') lcd.write(byte(spnRight));
      else if (scrmem[i] == '$') lcd.write(byte(backBut));
      else if (scrmem[i] == '@') lcd.write(byte(okBut));
      else if (scrmem[i] == '§') lcd.write(byte(celc));
      else if (scrmem[i] == '±') lcd.write(byte(celc));
      else lcd.write(scrmem[i]);
    } else lcd.write(' ');
  }
  txtOn = 0;
}

// Получить границы выделенного фрагмента
int getL(int pnt) {
  if (state == 'm') {
    int fst[] = {0, 7, 15};
    return fst[pnt];
  }
  if (state == 'n') {
    int fst[] = {0, 2, 8, 15};
    return fst[pnt];
  }
  if (state == 'd') {
    int fst[] = {0, 2, 6, 15};
    return fst[pnt];
  }
  if (state == 's') {
    int fst[] = {0, 1, 10};
    return fst[pnt];
  }
  if (state == 'r') {
    int fst[] = {0, 10};
    return fst[pnt];
  }
}
int getR(int pnt) {
  if (state == 'm') {
    int fst[] = {0, 12, 15};
    return fst[pnt];
  }
  if (state == 'n') {
    int fst[] = {0, 7, 13, 15};
    return fst[pnt];
  }
  if (state == 'd') {
    int fst[] = {0, 4, 11, 15};
    return fst[pnt];
  }
  if (state == 's') {
    int fst[] = {0, 7, 14};
    return fst[pnt];
  }
  if (state == 'r') {
    int fst[] = {9, 15};
    return fst[pnt];
  }
}

// Перейти к ближайшему pnt в направлении dir
void setPnt(int dir) {
  int qo;
  if (state == 'm') qo = 3;
  if (state == 'n') qo = 4;
  if (state == 'd') qo = 4;
  if (state == 's') qo = 3;
  if (state == 'r') qo = 2;
  pnt = (pnt + dir + qo) % qo;
  while (badPnt()) pnt = (pnt + (dir == 0 ? 1 : dir) + qo) % qo;
  l = getL(pnt);
  r = getR(pnt);
  rewrite();
}

// Обновить вторую строку в памяти
void updateScrMem() {
  if (state == 'm') {
    scrmem[0] = '^';
    scrmem[1] = ' ';
    scrmem[2] = '0' + (mods[mod][1] / 100);
    scrmem[3] = '0' + ((mods[mod][1] / 10) % 10);
    scrmem[4] = '0' + (mods[mod][1] % 10);
    scrmem[5] = '§';
    scrmem[6] = ' ';
    scrmem[7] = '<';
    if (diam == 1) {
      scrmem[8] = '±';
      scrmem[9] = '7';
      scrmem[10] = '5';
    } else {
      scrmem[8] = '3';
      scrmem[9] = '.';
      scrmem[10] = '0';
    }
    scrmem[11] = '>';
    scrmem[12] = ' ';
    scrmem[13] = ' ';
    scrmem[14] = ' ';
    scrmem[15] = '@';
  }
  if (state == 'n') {
    scrmem[0] = '^';
    scrmem[1] = ' ';
    scrmem[2] = '<';
    scrmem[3] = '0' + (mods[mod][1] / 100);
    scrmem[4] = '0' + ((mods[mod][1] / 10) % 10);
    scrmem[5] = '0' + (mods[mod][1] % 10);
    scrmem[6] = '§';
    scrmem[7] = '>';
    scrmem[8] = ' ';
    scrmem[9] = '<';
    if (diam == 1) {
      scrmem[10] = '±';
      scrmem[11] = '7';
      scrmem[12] = '5';
    } else {
      scrmem[10] = '3';
      scrmem[11] = '.';
      scrmem[12] = '0';
    }
    scrmem[13] = '>';
    scrmem[14] = ' ';
    scrmem[15] = '@';
  }
  if (state == 'd') {
    scrmem[0] = '^';
    scrmem[1] = ' ';
    scrmem[2] = (modq == 1) ? ' ' : '<';
    scrmem[3] = '0' + (mod - 7);
    scrmem[4] = (modq == 1) ? ' ' : '>';
    scrmem[5] = ':';
    scrmem[6] = ' ';
    scrmem[7] = '0' + (mods[mod][1] / 100);
    scrmem[8] = '0' + ((mods[mod][1] / 10) % 10);
    scrmem[9] = '0' + ((mods[mod][1] / 10) % 10);
    scrmem[10] = '§';
    scrmem[11] = ' ';
    scrmem[12] = ' ';
    scrmem[13] = ' ';
    scrmem[14] = ' ';
    scrmem[15] = '@';
  }
  if (state == 's') {
    scrmem[0] = '$';
    scrmem[1] = ' ';
    scrmem[2] = '[';
    scrmem[3] = 's';
    scrmem[4] = 'a';
    scrmem[5] = 'v';
    scrmem[6] = 'e';
    scrmem[7] = ']';
    scrmem[8] = ' ';
    scrmem[9] = ' ';
    scrmem[10] = '[';
    scrmem[11] = 'r';
    scrmem[12] = 'u';
    scrmem[13] = 'n';
    scrmem[14] = ']';
    scrmem[15] = ' ';
  }
  if (state == 'r') {
    scrmem[0] = '<';
    if (chDelChar == 1) {
      scrmem[1] = '0' + 2;
      scrmem[2] = '0' + 9;
      scrmem[3] = '0' + 0;
      scrmem[4] = '/';
      scrmem[5] = '0' + (mods[mod][1] / 100);
      scrmem[6] = '0' + ((mods[mod][1] / 10) % 10);
      scrmem[7] = '0' + (mods[mod][1] % 10);
      scrmem[8] = '§';
    } else {
      scrmem[1] = '0' + 2;
      scrmem[2] = '.';
      scrmem[3] = '0' + 9;
      scrmem[4] = '0' + 0;
      scrmem[5] = '/';
      if (diam == 1) {
        scrmem[6] = '±';
        scrmem[7] = '7';
        scrmem[8] = '5';
      } else {
        scrmem[6] = '3';
        scrmem[7] = '.';
        scrmem[8] = '0';
      }
    }
    scrmem[9] = '>';
    scrmem[10] = '[';
    scrmem[11] = 's';
    scrmem[12] = 't';
    scrmem[13] = 'o';
    scrmem[14] = 'p';
    scrmem[15] = ']';
  }
  setPnt(0);
}

// Отреагировать на взаимодействие с выделенным фрагментом
void editPnt(int dir = 0) {
  if (state == 'm') {
    if (pnt == 0) {
      if (dir == 0) {
        mod = 1;
      }
      if (dir == -1 && mod == firstMod()) {
        if (modq == 9) {
          state = 'd';
        } else {
          state = 'n';
        }
      }
      if (dir != 0 && mod != lastMod()) {
        setMod(dir);
      }
      if (dir == 1 && mod == lastMod()) {
        if (modq == 0) {
          state = 'n';
        } else {
          state = 'd';
        }
      }
      setUpState();
      return;
    }
    if (pnt == 1) {
      diam = (diam == 1) ? 2 : 1;
      updateScrMem();
      txtOn = 1;
      rewrite();
      return;
    }
    if (pnt == 2 && dir == 0) {
      state = 'r';
      setUpState();
      return;
    }
  }
  if (state == 'n') {
    if (pnt == 0) {
      if (dir == 0) {
        state = 'm';
        mod = 1;
      }
      if (dir == -1) {
        if (modq == 0) {
          state = 'n';
        } else {
          state = 'd';
        }
      }
      if (dir == 1) {
        state = 'm';
        mod = 1;
      }
      setUpState();
      return;
    }
    if (pnt == 1 && dir == 0) {
      if (dir == 0) {
        mods[0][1] = (mods[0][1] - 50 + 10) % 301 + 51;
      } else {
        mods[0][1] = (mods[0][1] - 50 + dir + 301) % 301 + 51;
      }
    }
    if (pnt == 2) {
      diam = (diam == 1) ? 2 : 1;
      updateScrMem();
      txtOn = 1;
      rewrite();
      return;
    }
    if (pnt == 3 && dir == 0) {
      state = 's';
      setUpState();
      return;
    }
  }
  if (state == 'd') {
    if (pnt == 0) {
      if (dir == 0) {
        state = 'm';
        mod = 1;
      }
      if (dir == -1) {
        state = 'm';
        mod = lastMod();
      }
      if (dir == 1) {
        if (modq == 9) {
          state = 'd';
        } else {
          state = 'n';
        }
      }
      setUpState();
      return;
    }
    if (pnt == 1 && dir != 0) {
      setMod(dir, true);
    }
    if (pnt == 2 && dir == 0) {
      delMod(mod);
      mod = 1;
      state = 'm';
      setUpState();
    }
    updateScrMem();
    rewrite();
    return;
  }
  if (state == 's') {
    if (pnt == 0 && dir == 0) {
      state = 'n';
      setUpState();
      return;
    }
    if (pnt == 1 && dir == 0) {
      ++modq;
      int newNum = firstFree();
      mods[newNum][0] = 1;
      mods[newNum][1] = mods[0][1];
      mods[newNum][2] = mods[0][2];
      modsBackUp();
      mod = newNum;
      state = 'm';
      setUpState();
      return;
    }
    if (pnt == 2 && dir == 0) {
      state = 'r';
      setUpState();
      return;
    }
  }
}

// Настроить все для нового выбранного состояния меню
void setUpState() {
  if (state == 'm') {
      lcd.setCursor(0, 0);
      lcd.print("                ");
      lcd.setCursor(0, 0);
      lcd.write(byte(branch));
    if (mod == 1) lcd.print("PLA");
    else if (mod == 2) lcd.print("PC");
    else if (mod == 3) lcd.print("PP");
    else if (mod == 4) lcd.print("PETG");
    else if (mod == 5) lcd.print("Neylon");
    else if (mod == 6) lcd.print("ABS");
    else if (mod == 7) lcd.print("ABS+PC");
    else {
      lcd.print("mode");
      lcd.print(mod - 7);
    }
      lcd.print(":");
    pnt = 0;
    updateScrMem();
    rewrite();
  }
  if (state == 'n') {
      lcd.setCursor(0, 0);
      lcd.print("                ");
    mod = 0;
      lcd.setCursor(0, 0);
      lcd.write(byte(branch));
      lcd.print("new mode ");
      lcd.print(firstFree());
      lcd.print(":");
    
    pnt = 0;
    updateScrMem();
    rewrite();
  }
  if (state == 'd') {
      lcd.setCursor(0, 0);
      lcd.print("                ");
    mod = firstMod();
    chDelChar = 1;
      lcd.setCursor(0, 0);
      lcd.print(byte(branch));
      lcd.print("delete mode");
    
    pnt = 0;
    updateScrMem();
    rewrite();
  }
  if (state == 's') {
      lcd.setCursor(0, 0);
      lcd.print("                ");
      lcd.setCursor(0, 0);
      lcd.write(byte(branch));
      lcd.print("new mode ");
      lcd.print(firstFree());
      lcd.print(byte(branch));
    
    pnt = 0;
    updateScrMem();
    rewrite();
  }
}

void setup() {
  Serial.begin(9600); // настройка потока для связи с компьютером


  // // Настройка подключенных устройств
  // scale.begin(DT, SCK); // весы
  lcd.begin(16, 2); // экран
  enc.setEncType(EB_HALFSTEP); // энкодер
  
  // // Настройка пинов
  pinMode(BUTR, INPUT); // лево
  pinMode(BUTL, INPUT); // право

  // pinMode(6, OUTPUT); ??? не очень понимаю, зачем нужно


  // Настройка спецсимволов
  lcd.createChar(spnr, spnr_);
  lcd.createChar(branch, branch_);
  lcd.createChar(okBut, okBut_);
  lcd.createChar(backBut, backBut_);
  lcd.createChar(spnRight, spnRight_);
  lcd.createChar(spnLeft, spnLeft_);
  lcd.createChar(celc, celc_);

  // Заполнить mods из постоянной памяти
  modsFill();

  state = 'm';
  mod = 1;
  setUpState();

  butrSt = 0;
  butlSt = 0;
}
 
void loop() {
  enc.tick();
  if (enc.right()) {
    editPnt(1);
  }
  if (enc.left()) {
    editPnt(-1);
  }
  if (enc.click()) {
    editPnt();
  }
  if (digitalRead(BUTR) == 1 && butrSt == 0) {
    setPnt(1);
  }
  if (digitalRead(BUTL) == 1 && butlSt == 0) {
    setPnt(-1);
  }
  butlSt = digitalRead(BUTL);
  butrSt = digitalRead(BUTR);
  delay(1);
  rewrite();
}