[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-f059dc9a6f8d3a56e377f745f24479a46679e63a5d9fe6f495e02850cd0d8118.svg)](https://classroom.github.com/online_ide?assignment_repo_id=7421866&assignment_repo_type=AssignmentRepo)
# RoboCup

La robocup es un evento creado para promover el acceso a tecnologías modernas, como la visión por ordenador, la comprensión del habla o el aprendizaje automático; y para impulsar el desarrollo de robots de servicio. Para la robocup de este año, hay tres pruebas, carry my luggage, find my mates y receptionist. Nosotros, para la entrega de esta práctica hemos decidido desarrollar codigo para las pruebas de carry my luggage y find my mates.

## Carry my luggage
En esta primera prueba, el robot está colocado fuera de la arena, y por medio

## Find my mates


## Como hemos abordado la práctica
Hemos decidido dividir las pruebas en varias tareas que habia que resolver. Cada una de ellas esta relacionada con un problema distinto de la práctica que había que solucionar para poder realizar la prueba. Había tres grandes partes, la vision, que era la encargada de detectar la informacion de los sensores; la navegacion, que tenia como objetivo desplazarse tanto a puntos concretos del mapa, como desplazarse a lo largo de el siguiendo a la persona, siendo capaz luego de volver al punto inicial.

### Visión
La parte de vision que hemos desarrollado tiene como objetivo captar toda la informacion que consideramos necesaria para poder realizar las pruebas, esto lo hacemos mediante uno de los sensores, la camara. Para ello, hemos utilizado opencv, que sirve tanto para la detección de movimiento, el reconocimiento de objetos, como para otras tareas.

El nodo mas importante dentro de esta parte es el llamado nodo_bbx_msg_publisher_advanced.cpp, puesto que con el pretendemos obtener el color de la camiseta, y tambien del pantalon, de la persona que tenemos delante. En primer lugar, intentamos obtener la imagen, y con ello las bounding boxes de una persona, por medio de darknet_ros, y a continuacion, recortamos un poco la imagen de la persona, para evitar tener problemas segun nuestra implementacion del filtrado.

En la deteccion de los colores, y ya teniendo unas bounding boxes que se ajustaban mas a la ropa, lo que hacemos, es averiguar la zona mas grande detectada con el mismo color. De esta forma, podemos intuir que la zona con la mayor superficie del mismo color va a corresponderse con la camiseta de la persona que hemos detectado. Asi mismo lo hacemos con la segunda zona mas grande del mismo color que detectamos, pero esta vez, lo que habremos detectado será el color del pantalon de la persona.

Por último, para poder utilizar la informacion que hemos obtenido mediante este nodo, lo que hacemos es, ya teniendo los colores, hemos creado un publicador en un topic llamado bbx_ropa, que va a publicar los colores de la camiseta y del pantalon de la persona. Esto nos es muy util, porque luego podemos ver esa informacion y guardarla para poder decirsela luego al operador.

```c++
void rgb_callback(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes, bbx_info *mensajero)
{
    cv_bridge::CvImagePtr img_ptr_rgb;
    std::string str_person ("person");

    int ancho = abs(mensajero->bbx_xmax - mensajero->bbx_xmin);
    int alto = abs(mensajero->bbx_ymax - mensajero->bbx_ymin);
    ROS_INFO("%d, %d", ancho, alto);
    
    //recortamos la imagen de la persona
    if(ancho > 0 && alto > 0){
      cv::Rect bbx_rect = cv::Rect(mensajero->bbx_xmin,mensajero->bbx_ymin,ancho,alto);
      cv::Mat bbx_img = cv::Mat(img_ptr_rgb->image, bbx_rect);
      cv::namedWindow( "person", cv::WINDOW_AUTOSIZE );
      cv::imshow( "person", bbx_img);
      detectar_color(bbx_img, mensajero);
    }
}

```
Esto es un trozo de codigo de este nodo, que tras detectar a la persona, recorta la imagen obtenida, y además llama a la funcion que se encarga de detectar los colores con la mayor superficie que encuentra.

### Navegación


### Diálogo

Para la parte de diálogo hemos utilizado una máquina de estados que tiene 3 estados posibles:

###### IDLE: 
En este estado solo pregunta el nombre del guest y pasa al siguuiente estado (LISTEN), además se crea una variable para almacemar la marca de tiempo.

###### LISTEN:
En este estado se empieza a escuchar durante máximo 15 segundos, si en 15 segundos no escucha la intención de obtener nombre, se pasa al sigiente estado (SPEAK).
Además de escuchar el nombre se comprueba si se lo ha dicho bien.
Para saber si se ha escuchado el nombre y si está bien escuchado, se utiliza dos variables booleanas.
- keep_listening: Es true si se escucha la intención de obtener nombre
- keep_listening_2 : Es true si se escucha la intención de comprobar nombre.
En el caso de que estas dos variables sean true, es que hemos obtenido el nombre. En caso contrario, pasado lod 15 segundos de escucha se vuelve a pregunatar por el nombre.

###### SPEAK:
Este caso lo que hace es volver a pasar al estado IDLE para que pregunte el nombre otra vez.

```c++
switch (state_)
    {
    
      case IDLE:
      
        ros::Duration(0.2).sleep();
        speak("Tell me your name");
        state_ = LISTEN;
        ROS_INFO("IDLE -> LISTEN");
        start_ts_ = ros::Time::now();
        

        break;

      case LISTEN:
        
        if ((ros::Time::now() - start_ts_).toSec() < WAITING_TIME) 
        {
          
          listen();
          
          if ( keep_listening_ == false) 
          {
            person_name_.name = name_;
              
            if (keep_listening_2_ == false)
            {
              ROS_INFO("NAME OBTAINED");
              pub_name_.publish(person_name_);
              stop_ = true;
              
            }
          }
          
        }
        
        else
        {
          state_ = SPEAK;
          ROS_INFO("LISTEN -> SPEAK");
        }
        
        break;

      case SPEAK:

        state_ = IDLE;
        break;
    }
```
Una vez que se ha obtenido el nombre, se publica por un topic.
