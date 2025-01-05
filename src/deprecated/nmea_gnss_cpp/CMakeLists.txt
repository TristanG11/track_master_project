cmake_minimum_required(VERSION 3.5)
project(nmea_gnss_cpp)

# Trouver les dépendances nécessaires
find_package(ament_cmake REQUIRED)        # Nécessaire pour ROS2
find_package(rclcpp REQUIRED)             # Client ROS2 en C++
find_package(sensor_msgs REQUIRED)        # Messages GNSS standard
find_package(msg_utils REQUIRED)          # Messages personnalisés
find_package(Boost REQUIRED COMPONENTS system) # Boost pour la gestion du port série
find_package(ament_cmake_gtest REQUIRED)
find_package(diagnostic_updater REQUIRED)


# Ajouter le répertoire des headers
include_directories(include)

# Créer une bibliothèque pour le parsing (nmea_parser)
add_library(nmea_parser_lib
  src/nmea_parser.cpp        # Implémentation des fonctions de parsing
)

# Lier les dépendances de la bibliothèque de parsing
target_link_libraries(nmea_parser_lib Boost::system)

# Créer une bibliothèque pour GNSS (nmea_gnss)
add_library(nmea_gnss_lib
  src/nmea_gnss.cpp          # Implémentation des fonctions GNSS
)

# Lier la bibliothèque de parsing et les dépendances de la bibliothèque GNSS
target_link_libraries(nmea_gnss_lib nmea_parser_lib)
ament_target_dependencies(nmea_gnss_lib rclcpp sensor_msgs msg_utils Boost diagnostic_updater)

# Ajouter l'exécutable principal
add_executable(nmea_gnss_node
  src/nmea_gnss_node.cpp     # Fichier contenant le main
)

# Lier les bibliothèques et dépendances à l'exécutable
target_link_libraries(nmea_gnss_node nmea_gnss_lib nmea_parser_lib)
ament_target_dependencies(nmea_gnss_node rclcpp sensor_msgs msg_utils Boost diagnostic_updater)


# Install the launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
    # Ajouter un exécutable pour les tests
    ament_add_gtest(parser_test
        test/parser_test.cpp
    )

    # Lier les bibliothèques nécessaires
    target_link_libraries(parser_test
        nmea_gnss_lib
        nmea_parser_lib
    )
endif()

# Installer l'exécutable
install(TARGETS
  nmea_gnss_node             # Exécutable principal
  nmea_gnss_lib              # Bibliothèque GNSS
  nmea_parser_lib            # Bibliothèque Parser
  DESTINATION lib/${PROJECT_NAME}
)

# Installer les headers
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
