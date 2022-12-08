#include <Ultrasonic.h>
 
/* Definições dos GPIOs para leitura do sensor ultrasonico */
#define GPIO_TRIGGER    12
#define GPIO_ECHO   11
 
/* Definições de operação do sensor ultrasônico */
#define MINIMUM_DISTANCE 10.0 //cm
#define TIME_BETWEEN_READINGS   250  //ms
 
/* Definições para controle dos dois motores */
#define IN_1    3
#define IN_2    4
#define IN_3    5
#define IN_4    6
 
/* Definições dos motores a serem controlados */
#define MOTOR_A 0x00
#define MOTOR_B 0x01
 
/* Definições das ações dos motores */
#define BREAK    0x00
#define ANTI_CLOCKWISE_MOVE  0x01
#define CLOCKWISE_MOVE   0x02
#define STOP   0x03
 
/* Definições de sentido de giro (em caso de obstáculo) */
#define ANTI_CLOCKWISE  0x00
#define CLOCKWISE_TURN  0x01
 
/* Definições do desvio de objetos */
#define WAITING_OBSTACLE    0x00
#define TURNING 0x01
 
/* Variáveis e objetos globais */
Ultrasonic ultrasonic(GPIO_TRIGGER, GPIO_ECHO);
char last_side_turned = ANTI_CLOCKWISE;
char state_obstacle_freed = WAITING_OBSTACLE;
 
/* Protótipos */
void motor_gpio_config(void);
void motor_control(char motor, char action);
float read_distance(void);
void state_machine_obstacle(float obstacle_distance);
 
/* Função: configura GPIOs de controle do L298N como output
 * Parâmetros: nenhum
 * Retorno: nenhum
 */
void motor_gpio_config(void)
{
    pinMode(IN_1, OUTPUT);
    pinMode(IN_2, OUTPUT);
    pinMode(IN_3, OUTPUT);
    pinMode(IN_4, OUTPUT);
}
 
/* Função: controle um motor (freia, movimento anti-horário, movimento horário
 *         ou ponto morto)
 * Parâmetros: motor a ser controlado e ação desejada
 * Retorno: nenhum
 */
void motor_control(char motor, char action)
{
    int gpio_1_motor = 0;
    int gpio_2_motor = 0;
 
    /* seleciona os GPIOs de acordo com o motor desejado */
    switch(motor)
    {
        case MOTOR_A:
            gpio_1_motor = IN_1;
            gpio_2_motor = IN_2;
            break;
     
        case MOTOR_B:
            gpio_1_motor = IN_3;
            gpio_2_motor = IN_4;
            break;
 
        default:
            /* Motor inválido. Nada mais deve ser feito nesta função */
            return;            
    }
 
    /* Controla o motor conforme ação desejada */
    switch(action)
    {
        case BREAK:
            digitalWrite(gpio_1_motor, HIGH);
            digitalWrite(gpio_2_motor, HIGH);
            break;
 
        case ANTI_CLOCKWISE_MOVE:
            digitalWrite(gpio_1_motor, LOW);
            digitalWrite(gpio_2_motor, HIGH);
            break;
 
        case CLOCKWISE_MOVE:
            digitalWrite(gpio_1_motor, HIGH);
            digitalWrite(gpio_2_motor, LOW);
            break;
 
        case STOP:
            digitalWrite(gpio_1_motor, LOW);
            digitalWrite(gpio_2_motor, LOW);
            break;
 
        default:
            /* Ação inválida. Nada mais deve ser feito nesta função */
            return;                                                            
    }    
}
 
/* Função: faz leitura da distância (em centímetros) de obstáculo a frente do robô
 * Parâmetros: nenhum
 * Retorno: distância (cm)
 */
float read_distance(void)
{
    float distance = 0.0;
    long microsec = 0;
     
    microsec = ultrasonic.timing();
    distance = ultrasonic.convert(microsec, Ultrasonic::CM);
    return distance;
}
 
/* Função: maquina de estado responsavel por controlar o desvio de obstáculos
 * Parâmetros: distância de obstáculo a frente
 * Retorno: nenhum
 */
void state_machine_obstacle(float obstacle_distance)
{
    switch(state_obstacle_freed)
    {
        case WAITING_OBSTACLE:
            if (obstacle_distance <= MINIMUM_DISTANCE)
            {
                /* Obstáculo encontrado. O robô deve girar para
                   desviar dele */
                Serial.println("[MOVIMENTO] Obstaculo encontrado!");   
                 
                /* Alterna sentido de giro para se livrar de obstáculos
                   (para otimizar o desvio de obstáculos) */
                if (last_side_turned == ANTI_CLOCKWISE)
                    last_side_turned = CLOCKWISE_TURN;
                else
                    last_side_turned = ANTI_CLOCKWISE;
                     
                state_obstacle_freed = TURNING; 
            }
            else
            {
                Serial.println("[MOVIMENTO] Sem obstaculos a frente");
                 
                /* Se não há obstáculos, continua em frente */
                motor_control(MOTOR_A, CLOCKWISE_MOVE);
                motor_control(MOTOR_B, CLOCKWISE_MOVE);
            }
             
            break;
 
        case TURNING: 
            if (obstacle_distance > MINIMUM_DISTANCE)
            {
                /* Não há mais obstáculo a frente do robô */  
                state_obstacle_freed = WAITING_OBSTACLE; 
            }
            else
            {
                if (last_side_turned == ANTI_CLOCKWISE)
                {
                    motor_control(MOTOR_A, ANTI_CLOCKWISE_MOVE);
                    motor_control(MOTOR_B, CLOCKWISE_MOVE);
                    Serial.println("[MOVIMENTO] Girando no sentido anti-horario...");
                }
                else
                {
                    motor_control(MOTOR_A, CLOCKWISE_MOVE);
                    motor_control(MOTOR_B, ANTI_CLOCKWISE_MOVE);
                    Serial.println("[MOVIMENTO] Girando no sentido horario...");
                }
            }
             
            break;
    }
}
 
void setup() 
{
    Serial.begin(115200);
     
    /* Configura GPIOs de controle do L298N como output e coloca motor em condição de freio */
    motor_gpio_config();    
    motor_control(MOTOR_A, BREAK);
    motor_control(MOTOR_B, BREAK);
}
 
void loop() 
{
    float forward_distance = 0.0;
 
    forward_distance = read_distance();
    Serial.print("* Distancia lida: ");
    Serial.print(forward_distance);
    Serial.println("cm");
 
    /* Verifica se há obstáculo a frente */
    state_machine_obstacle(forward_distance);
 
    delay(TIME_BETWEEN_READINGS);
}