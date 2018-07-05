# TFM

## Histórico

### 1.0 Primera versión
El controlador puede modificar la velocidad de un rotor para aumentarla o reducirla en un valor prefijado. El estado del sistema viene dado por la posición del quadrotor y por la posición del objetivo

### 1.1 Añadida rotación
Se añade la rotación a las observaciones.

### 1.2 Modificados los parámetros
Se modifican los parámetros parametrizando el tiempo que se ejecuta la simulación y el valor con el que se modifica la velocidad de los rotores

### 2.0 Control "realista"
Se modifica el control para que cada acción modifique los valores de pitch, yaw, roll y throttle.

### 2.1 Agente DQN
Se cambia el agente de CEM a DQN

### 3.0 Estabilización
Se añade la estabilización mediante un nuevo objeto y el control PID original del quadrotor

### 3.1 Espacio de observación 
Se modifica el espacio de observación para limitarlo al espacio real posible

### 4.0 Se añade la distancia
La distancia, que en teoría la RNA podría "aprender" a verla, se añade como observación

### 4.1 Se descompone la distancia
Se descompone la distancia en X,Y,Z y se deja con signo. De esta forma se espera que sea capaz de deducir la dirección y el sentido que debe tomar.