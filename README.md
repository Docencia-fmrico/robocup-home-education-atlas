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

