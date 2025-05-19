#include <stdio.h>
#include <string.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/supervisor.h>

#define TIME_STEP 512
#define QtddSensoresProx 8
#define QtddLeds 10
#define QtddCaixa 20

int main(int argc, char **argv) {
  double LeituraSensorProx[QtddSensoresProx];
  double AceleradorDireito = 1.0, AceleradorEsquerdo = 1.0;
  bool EncontrouCaixaLeve = false;
  WbNodeRef caixa[QtddCaixa];
  char nomeCaixa[20] = {0};

  double ultimaPosX = 0.0, ultimaPosZ = 0.0;
  int tempoParado = 0;
  int direcaoEvasao = 0;
  int contadorTentativas = 0;

  bool rotacionando = false;
  int tempoRotacao = 0;  // para controlar tempo de rotação

  wb_robot_init();

  WbDeviceTag MotorEsquerdo = wb_robot_get_device("left wheel motor");
  WbDeviceTag MotorDireito = wb_robot_get_device("right wheel motor");

  wb_motor_set_position(MotorEsquerdo, INFINITY);
  wb_motor_set_position(MotorDireito, INFINITY);
  wb_motor_set_velocity(MotorEsquerdo, 0);
  wb_motor_set_velocity(MotorDireito, 0);

  WbDeviceTag SensorProx[QtddSensoresProx];
  char nomeSensor[4];
  for (int i = 0; i < QtddSensoresProx; i++) {
    sprintf(nomeSensor, "ps%d", i);
    SensorProx[i] = wb_robot_get_device(nomeSensor);
  }

  for (int c = 0; c < QtddCaixa; c++) {
    sprintf(nomeCaixa, "CAIXA%d", c + 1);
    caixa[c] = wb_supervisor_node_get_from_def(nomeCaixa);
    if (caixa[c] != NULL)
      printf("%d. %s\n", c + 1, nomeCaixa);
    else
      printf("Não foi possível carregar a caixa %s\n", nomeCaixa);
  }
  printf("\n\n CAIXAS OK  \n\n");

  for (int i = 0; i < QtddSensoresProx; i++) {
    wb_distance_sensor_enable(SensorProx[i], TIME_STEP);
  }

  WbDeviceTag Leds[QtddLeds];
  Leds[0] = wb_robot_get_device("led0");
  wb_led_set(Leds[0], -1);

  const double *posicaoRobo = wb_supervisor_node_get_position(wb_supervisor_node_get_self());
  ultimaPosX = posicaoRobo[0];
  ultimaPosZ = posicaoRobo[2];

  while (wb_robot_step(TIME_STEP) != -1) {
    double PosicaoCaixaAntes[QtddCaixa][3];

    for (int i = 0; i < QtddSensoresProx; i++) {
      LeituraSensorProx[i] = wb_distance_sensor_get_value(SensorProx[i]) - 60;
    }

    for (int c = 0; c < QtddCaixa; c++) {
      const double *pos = wb_supervisor_node_get_position(caixa[c]);
      PosicaoCaixaAntes[c][0] = pos[0];
      PosicaoCaixaAntes[c][1] = pos[1];
      PosicaoCaixaAntes[c][2] = pos[2];
    }

    wb_led_set(Leds[0], wb_led_get(Leds[0]) * -1);

    const double *posicaoAtual = wb_supervisor_node_get_position(wb_supervisor_node_get_self());
    if (!EncontrouCaixaLeve && fabs(posicaoAtual[0] - ultimaPosX) < 0.001 && fabs(posicaoAtual[2] - ultimaPosZ) < 0.001) {
      tempoParado += TIME_STEP;
    } else {
      tempoParado = 0;
      contadorTentativas = 0;
    }

    ultimaPosX = posicaoAtual[0];
    ultimaPosZ = posicaoAtual[2];

    if (EncontrouCaixaLeve) {
      // Se encontrou a caixa leve, inicia rotação no próprio eixo
      if (!rotacionando) {
        rotacionando = true;
        tempoRotacao = 0;
        printf("Caixa leve encontrada.\n");
      }

      if (rotacionando) {
        // Velocidades opostas para girar no próprio eixo
        wb_motor_set_velocity(MotorEsquerdo, 3.0);
        wb_motor_set_velocity(MotorDireito, -3.0);

        tempoRotacao += TIME_STEP;

        // Rotaciona por aproximadamente 3 segundos
        if (tempoRotacao >= 3000) {
          // Para os motores e finaliza execução
          wb_motor_set_velocity(MotorEsquerdo, 0);
          wb_motor_set_velocity(MotorDireito, 0);
          printf("Finalizar rotação\n");
          wb_robot_cleanup();
          return 0;
        }
      }
    } else {
      // Comportamento normal do robô (evasão de obstáculos e movimento)
      if (tempoParado >= 1000) {
        printf("Robô parado por  01 segundo. Mudando direção. Tentativa: %d\n", ++contadorTentativas);

        if (contadorTentativas > 3) {
          direcaoEvasao = (direcaoEvasao + 1) % 4;
          contadorTentativas = 0;
        }

        switch (direcaoEvasao) {
          case 0:
            AceleradorDireito = 0.1;
            AceleradorEsquerdo = 1.0;
            break;
          case 1:
            AceleradorDireito = 1.0;
            AceleradorEsquerdo = 0.1;
            break;
          case 2:
            AceleradorDireito = -0.5;
            AceleradorEsquerdo = 1.0;
            break;
          case 3:
            AceleradorDireito = 1.0;
            AceleradorEsquerdo = -0.5;
            break;
        }

        tempoParado = 0;
      } else if ((LeituraSensorProx[7] > 1000 || LeituraSensorProx[0] > 1000 || LeituraSensorProx[1] > 1000) && (LeituraSensorProx[2] > 1000 || LeituraSensorProx[3] > 1000)) {
        AceleradorDireito = 1;
        AceleradorEsquerdo = -1;
      } else if ((LeituraSensorProx[7] > 1000 || LeituraSensorProx[0] > 1000 || LeituraSensorProx[6] > 1000) && (LeituraSensorProx[4] > 1000 || LeituraSensorProx[5] > 1000)) {
        AceleradorDireito = -1;
        AceleradorEsquerdo = 1;
      } else if (LeituraSensorProx[7] > 1000 || LeituraSensorProx[0] > 1000) {
        AceleradorDireito = -1;
        AceleradorEsquerdo = 1;
      } else {
        AceleradorDireito = 1;
        AceleradorEsquerdo = 1;
      }

      if (AceleradorDireito < 0 && AceleradorEsquerdo < 0) {
        if (direcaoEvasao % 2 == 0) {
          AceleradorDireito = 0.2;
          AceleradorEsquerdo = 1.0;
        } else {
          AceleradorDireito = 1.0;
          AceleradorEsquerdo = 0.2;
        }
      }

      wb_motor_set_velocity(MotorEsquerdo, 6.28 * AceleradorEsquerdo);
      wb_motor_set_velocity(MotorDireito, 6.28 * AceleradorDireito);
    }

    wb_robot_step(TIME_STEP);

    for (int c = 0; c < QtddCaixa; c++) {
      const double *PosicaoCaixaDepois = wb_supervisor_node_get_position(caixa[c]);

      if (fabs(PosicaoCaixaAntes[c][0] - PosicaoCaixaDepois[0]) > 0.001 ||
          fabs(PosicaoCaixaAntes[c][1] - PosicaoCaixaDepois[1]) > 0.001 ||
          fabs(PosicaoCaixaAntes[c][2] - PosicaoCaixaDepois[2]) > 0.001) {
        if (!EncontrouCaixaLeve) {
          EncontrouCaixaLeve = true;
        }
        break;
      }
    }
  }

  wb_robot_cleanup();
  return 0;
}
