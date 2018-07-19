# Система управления мобильным роботом. 

## 1. Описание системы управления

### 1.1 Прототип гусеничного робота

Прототип гусеничного робота пресдставлен на рисунке 1.

![Иллюстрация к проекту](https://github.com/PonomarevDA/MobileRobotControlSystem/blob/master/img/robot.PNG)

Рисунок 1 - Прототип гусеничного робота

### 1.2 Аппаратная часть системы управления

Аппаратная часть системы управления продемонстрирована на рисунке 2.

![Иллюстрация к проекту](https://github.com/PonomarevDA/MobileRobotControlSystem/blob/master/img/%D0%A1%D1%82%D1%80%D1%83%D0%BA%D1%82%D1%83%D1%80%D0%BD%D0%B0%D1%8F%20%D1%81%D1%85%D0%B5%D0%BC%D0%B0.PNG)

Рисунок 2 - Аппаратная часть системы управления

### 1.3 Структурная схема системы управления

Структурная схема системы управления представлена на рисунке 3.

![Иллюстрация к проекту](https://github.com/PonomarevDA/MobileRobotControlSystem/blob/master/img/struct_circuit.png)

Рисунок 3 - Структурная схема системы управления

Как видно из струкурной схемы, в системе управления используются 2 следующих регулятора:

1. ПИ-регулятор, поддерживающий вращение двигателей с одинаковой скоростью.

2. Регулятор плавного изменения скажности ШИМ (~ скорости).

Контура по скорости нет, это означает, что вращение вала двигателей задается напрямую скажностью ШИМ.

### 1.4 ПИ-регулятор

Результаты работы ПИ-регулятора продемонстрированы на рисунках 4 и 5.

![Иллюстрация к проекту](https://github.com/PonomarevDA/MobileRobotControlSystem/blob/master/img/test_pi_regulator1.PNG)

Рисунок 4 - Показание энкодеров при прямолинейном движении робота вперед на 75 см.

![Иллюстрация к проекту](https://github.com/PonomarevDA/MobileRobotControlSystem/blob/master/img/test_pi_regulator2.PNG)

Рисунок 5 - Показание энкодеров при прямолинейном движении робота вперед на 75 см.

Вывод: использование ПИ-регулятора оправдано.

### 1.5 Плавное изменение скорости

Плавное изменение скорости происходит в 2 этапа:

1. Сначала происходит ускорение робота. Это необходимо для уменьшение пускового тока.

2. Затем происходит замедление робота. Это необходимо для того, чтобы уменьшить влияние инерции на точность системы, 
которое возможно при резком торможении робота на большой скорости.

На практике скорость робота не превышает 20 см/сек, поэтому замедление себя никак не проявило. Однако его можно 
использовать на более мощных роботах.

## 2 Описание файловой архитектуры программы

Файловая архитектура продемонстрирована на схеме на рисунке 6.

![Иллюстрация к проекту](https://github.com/PonomarevDA/MobileRobotControlSystem/blob/master/img/program_architecture.png)

Рисунок 6 - Файловая архитектура проекта

Как видно из данной архитектуры, все файлы разделены на следующие категории:

1. Файл hard, который содержит в себе реализацию доступа к переферии микроконтроллера.

2. Файлы encder, motor_control, soft_timer, rangefinder являются устройствами, с которыми взаимодействует микроконтроллер. 
UART, конечно же, является переферией микроконтроллера и его логично разместить в первую группу. Однако он, в тоже время, 
является интерфейсом для связи мк с другими устройствами, например, с компьютером. Поскольку для этой связи достаточно
уровня абстракции, предоставляемого данным модулем, он был размещен в данную категорию.

3. Файлы robot_control и robot_test реализуют и тестируют соответственно систему управления роботом.

4. Файлы math и text были размещены в дополнительную группу, поскольку больше никуда не подходят.