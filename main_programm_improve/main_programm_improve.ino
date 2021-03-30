#include <QTRSensors.h>//初始化QTR库
QTRSensors qtr;
const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];
//备注：电机控制4（方向）,5（转速）高电平，右轮（一号电机）反转（倒车）；电机控制6（转速）,7（方向）高电平，左轮（二号电机）正转（前进）
//自定义变量
//定义：右电机代号1，左电机代号2
int Y = 1;
int Z = 2;
int t = 10; //制动时间(ms)
int max_speed = 60; //最大速度
int yz_IR = 1200; //红外感光器阈值
int jiancha = 10; //系统自检时间
float d = max_speed / (t / jiancha); //每次速度变化幅度
float yz_gm = 50; //光敏电阻触发阈值（电压）
int pin_gm = A0; //光敏接口针脚
int ini_gm = 200;
int zt = 0; //状态数，初始为0（关闭）
int zt_z2 = 0;
int zt_z1 = 0;
int zt_y1 = 0;
int zt_y2 = 0;
int w_r_speed;
int qibu_t = 10;
int qibu_v = 254;
//自定义函数
int zt_change(float v_gm, float yz_gm, int zt) {
  if (v_gm >= yz_gm) { //没有额外照明（电阻大，电压高）
    return zt;
  }
  else { //有额外照明(v_gm<yz_gm)（电阻小，电压低）
    if (zt = 0) {
      return 1;
    } else {
      return 0;
    }
  }
}
void setup() {
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {
    8, 9, 10, 11
  }, SensorCount);
  Serial.begin(9600);
  //左轮针脚
  pinMode(6, OUTPUT);//方向
  pinMode(7, OUTPUT);//转速
  //右轮针脚
  pinMode(4, OUTPUT);//方向
  pinMode(5, OUTPUT);//转速
  //光敏针脚
  pinMode(pin_gm, INPUT);

}

void loop() {
  //  float gm = analogRead(pin_gm); //读取光敏针脚
  //  Serial.print(gm);
  //  Serial.print('\t');
  //  Serial.println();
  //  if (gm >= yz_gm) { //没有额外照明（电阻大，电压高）
  //    zt = zt;
  //  }
  //  else if (gm<yz_gm and ini_gm>yz_gm) { //有额外照明(gm<yz_gm)（电阻小，电压低）
  //    if (zt = 0) {
  //      return 1;
  //    } else {
  //      return 0;
  //    }
  //  }
  //  ini_gm = gm;
  zt = 1;
  switch (zt) {
    case (0): //不动
      delay(jiancha);
      break;
    case 1://动
      qtr.read(sensorValues);
      for (uint8_t i = 0; i < SensorCount; i++)
      {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
      }
      Serial.println();
      if (sensorValues[0] >= yz_IR) {
        zt_z2 = 1;
      } else {
        zt_z2 = 0;
      }
      if (sensorValues[1] >= yz_IR) {
        zt_z1 = 1;
      } else {
        zt_z1 = 0;
      }
      if (sensorValues[2] >= yz_IR) {
        zt_y1 = 1;
      } else {
        zt_y1 = 0;
      }
      if (sensorValues[3] >= yz_IR) {
        zt_y2 = 1;
      } else {
        zt_y2 = 0;
      }
      Serial.print(zt_z2);
      Serial.print('\t');
      Serial.print(zt_z1);
      Serial.print('\t');
      Serial.print(zt_y1);
      Serial.print('\t');
      Serial.print(zt_y2);
      Serial.print('\t');
      Serial.println();
      //检测传感器状态,左2（），左1（），右1（），右2（）
      if (zt_z2 == 0 and zt_z1 == 0 and zt_y1 == 0 and zt_y2 == 0) {
        //左2（0），左1（0），右1（0），右2（0）→前进
        //        digitalWrite(4, LOW);
        //        analogWrite(5, qibu_v);
        //        analogWrite(6, qibu_v);
        //        digitalWrite(7, HIGH);
        //        delay(qibu_t);
        digitalWrite(4, LOW);
        analogWrite(5, max_speed);
        analogWrite(6, max_speed);
        digitalWrite(7, HIGH);
        Serial.print("前进");
        Serial.print('\t');
        Serial.println();
        delay(jiancha);
      }
      else if (zt_z2 == 0 and zt_z1 == 1 and zt_y1 == 0 and zt_y2 == 0) {
        //左2（0），左1（1），右1（0），右2（0）→左轮制动，左转
        digitalWrite(4, LOW);
        analogWrite(5, max_speed);
        digitalWrite(7, HIGH);
        float r_speed = max_speed;
        while (zt_z2 == 0 and zt_z1 == 1 and zt_y1 == 0 and zt_y2 == 0 and r_speed >= 0) {
          qtr.read(sensorValues);
          if (sensorValues[0] >= yz_IR) {
            zt_z2 = 1;
          } else {
            zt_z2 = 0;
          }
          if (sensorValues[1] >= yz_IR) {
            zt_z1 = 1;
          } else {
            zt_z1 = 0;
          }
          if (sensorValues[2] >= yz_IR) {
            zt_y1 = 1;
          } else {
            zt_y1 = 0;
          }
          if (sensorValues[3] >= yz_IR) {
            zt_y2 = 1;
          } else {
            zt_y2 = 0;
          }
          w_r_speed = int(r_speed);
          analogWrite(6, w_r_speed);
          r_speed = r_speed - d;
          delay(jiancha);
        }
        if (r_speed < 0) {
          while (zt_z2 == 0 and zt_z1 == 1 and zt_y1 == 0 and zt_y2 == 0) {
            qtr.read(sensorValues);
            if (sensorValues[0] >= yz_IR) {
              zt_z2 = 1;
            } else {
              zt_z2 = 0;
            }
            if (sensorValues[1] >= yz_IR) {
              zt_z1 = 1;
            } else {
              zt_z1 = 0;
            }
            if (sensorValues[2] >= yz_IR) {
              zt_y1 = 1;
            } else {
              zt_y1 = 0;
            }
            if (sensorValues[3] >= yz_IR) {
              zt_y2 = 1;
            } else {
              zt_y2 = 0;
            }
            analogWrite(6, 0);
            delay(jiancha);
          }
        }
      }
      else if (zt_z2 == 1 and zt_z1 == 1 and zt_y1 == 0 and zt_y2 == 0) {
        //左2（1），左1（1），右1（0），右2（0）→左轮反向转动，原地左转
        digitalWrite(4, LOW);
        analogWrite(5, max_speed);
        digitalWrite(7, LOW);
        float r_speed = 0;
        while (zt_z2 == 1 and zt_z1 == 1 and zt_y1 == 0 and zt_y2 == 0 and r_speed <= max_speed) {
          qtr.read(sensorValues);
          if (sensorValues[0] >= yz_IR) {
            zt_z2 = 1;
          } else {
            zt_z2 = 0;
          }
          if (sensorValues[1] >= yz_IR) {
            zt_z1 = 1;
          } else {
            zt_z1 = 0;
          }
          if (sensorValues[2] >= yz_IR) {
            zt_y1 = 1;
          } else {
            zt_y1 = 0;
          }
          if (sensorValues[3] >= yz_IR) {
            zt_y2 = 1;
          } else {
            zt_y2 = 0;
          }
          w_r_speed = int(r_speed);
          analogWrite(6, w_r_speed);
          r_speed = r_speed + d;
          delay(jiancha);
        }
        if (r_speed > max_speed) {
          while (zt_z2 == 1 and zt_z1 == 1 and zt_y1 == 0 and zt_y2 == 0) {
            qtr.read(sensorValues);
            if (sensorValues[0] >= yz_IR) {
              zt_z2 = 1;
            } else {
              zt_z2 = 0;
            }
            if (sensorValues[1] >= yz_IR) {
              zt_z1 = 1;
            } else {
              zt_z1 = 0;
            }
            if (sensorValues[2] >= yz_IR) {
              zt_y1 = 1;
            } else {
              zt_y1 = 0;
            }
            if (sensorValues[3] >= yz_IR) {
              zt_y2 = 1;
            } else {
              zt_y2 = 0;
            }
            analogWrite(6, max_speed);
            delay(jiancha);
          }
        }
      }
      else if (zt_z2 == 0 and zt_z1 == 0 and zt_y1 == 1 and zt_y2 == 0) {
        //左2（0），左1（0），右1（1），右2（0）→右轮制动，右转
        analogWrite(6, max_speed);
        digitalWrite(7, HIGH);
        digitalWrite(4, LOW);
        float r_speed = max_speed;
        while (zt_z2 == 0 and zt_z1 == 0 and zt_y1 == 1 and zt_y2 == 0 and r_speed >= 0) {
          qtr.read(sensorValues);
          if (sensorValues[0] >= yz_IR) {
            zt_z2 = 1;
          } else {
            zt_z2 = 0;
          }
          if (sensorValues[1] >= yz_IR) {
            zt_z1 = 1;
          } else {
            zt_z1 = 0;
          }
          if (sensorValues[2] >= yz_IR) {
            zt_y1 = 1;
          } else {
            zt_y1 = 0;
          }
          if (sensorValues[3] >= yz_IR) {
            zt_y2 = 1;
          } else {
            zt_y2 = 0;
          }
          w_r_speed = int(r_speed);
          analogWrite(5, w_r_speed);
          r_speed = r_speed - d;
          delay(jiancha);
        }
        if (r_speed < 0) {
          while (zt_z2 == 0 and zt_z1 == 0 and zt_y1 == 1 and zt_y2 == 0) {
            qtr.read(sensorValues);
            if (sensorValues[0] >= yz_IR) {
              zt_z2 = 1;
            } else {
              zt_z2 = 0;
            }
            if (sensorValues[1] >= yz_IR) {
              zt_z1 = 1;
            } else {
              zt_z1 = 0;
            }
            if (sensorValues[2] >= yz_IR) {
              zt_y1 = 1;
            } else {
              zt_y1 = 0;
            }
            if (sensorValues[3] >= yz_IR) {
              zt_y2 = 1;
            } else {
              zt_y2 = 0;
            }
            analogWrite(5, 0);
            delay(jiancha);
          }
        }
      }
      else if (zt_z2 == 0 and zt_z1 == 0 and zt_y1 == 1 and zt_y2 == 1) {
        //左2（0），左1（0），右1（1），右2（1）→右轮反向转动，原地右转
        analogWrite(6, max_speed);
        digitalWrite(7, HIGH);
        digitalWrite(4, HIGH);
        float r_speed = 0;
        while (zt_z2 == 0 and zt_z1 == 0 and zt_y1 == 1 and zt_y2 == 1 and r_speed <= max_speed) {
          qtr.read(sensorValues);
          if (sensorValues[0] >= yz_IR) {
            zt_z2 = 1;
          } else {
            zt_z2 = 0;
          }
          if (sensorValues[1] >= yz_IR) {
            zt_z1 = 1;
          } else {
            zt_z1 = 0;
          }
          if (sensorValues[2] >= yz_IR) {
            zt_y1 = 1;
          } else {
            zt_y1 = 0;
          }
          if (sensorValues[3] >= yz_IR) {
            zt_y2 = 1;
          } else {
            zt_y2 = 0;
          }
          w_r_speed = int(r_speed);
          analogWrite(5, w_r_speed);
          r_speed = r_speed + d;
          delay(jiancha);
        }
        if (r_speed > max_speed) {
          while (zt_z2 == 0 and zt_z1 == 0 and zt_y1 == 1 and zt_y2 == 1) {
            qtr.read(sensorValues);
            if (sensorValues[0] >= yz_IR) {
              zt_z2 = 1;
            } else {
              zt_z2 = 0;
            }
            if (sensorValues[1] >= yz_IR) {
              zt_z1 = 1;
            } else {
              zt_z1 = 0;
            }
            if (sensorValues[2] >= yz_IR) {
              zt_y1 = 1;
            } else {
              zt_y1 = 0;
            }
            if (sensorValues[3] >= yz_IR) {
              zt_y2 = 1;
            } else {
              zt_y2 = 0;
            }
            analogWrite(5, max_speed);
            delay(jiancha);
          }
        }
      } else {
        digitalWrite(4, LOW);
        analogWrite(5, max_speed);
        analogWrite(6, max_speed);
        digitalWrite(7, HIGH);
        delay(jiancha);
      }
      //左2（0），左1（0），右1（0），右2（0）→前进
      //左2（0），左1（1），右1（0），右2（0）→左轮制动，左转
      //左2（1），左1（1），右1（0），右2（0）→左轮反向转动，原地左转
      //左2（0），左1（0），右1（1），右2（0）→右轮制动，右转
      //左2（0），左1（0），右1（1），右2（1）→右轮反向转动，原地右转
      break;
  }
}
