# Informe final Robotica

### Tabla de párametros DH y cinemática Directa (incluye un reultado númerico).
### Cálculo de la Cinemática Inversa del pincher (incluye un resultado númerico).
## Descripción de la solución creada, el proceso de preparación y programación (hay que ser detallado, podemos usar los vídeos del whatsapp en esta parte).

Para la solución del problema decidimos usar un paquete de ROS2 que crease un nodo tipo publisher, el cual publica un mensaje  un tópico dado en base a los comandos recibidos desde el joystick. Una vez el nodo envía los mensajes con el formato correcto podemos configurar otro nodo que este suscrito a ese  tópico y mueva una simulación en coppelia usando los comandos recibidos. De la misma forma para mover el robot físico se utiliza el paquete de dynamixel sdk para crear un segundo nodo suscrito al tópico y enviarle las mismas instrucciones de comando

## Diagrama de flujo de las acciones del robot.
```mermaid
flowchart TD
    A(Inicio) --> B((Auto/Manual))

    B -->|Automático| C(Modo Automático)
    B -->|Manual| D(Modo Manual)
    
    C --> E(Seleccionar trayectoria)
    E --> F(Ejecutar rutina preprogramada)
    F --> G(Visualizar en GUI)
    
    D --> H(Control con Joystick)
    H --> I(Modificar posición y orientación)
    I --> J(Abrir/Cerrar gripper)
    
    G --> K(Enviar comandos por LAN)
    J --> K
    K --> L(Ejecutar en Máquina Remota)
    L --> M(Visualización de Cámara)
    M --> N(Fin del proceso)
```
### Plano de planta y descripción de la teleoperación.
### Código en Matlab o Python de la solución.
### Comparación objetiva de la teleoperación manual y automática realizada los robots.
### Vídeo de la implementación (Incluir la intro del LabSIR: https://drive.google.com/file/d/1wSxw7m7n5hXOtkc8C0H0lLAxTx3BqQSe/view).
