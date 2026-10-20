"""Pont ROS 2 du rover FieldMind : client de la WebSocket /ws/ros du logiciel du rover.

- `conversions` : JSON de /ws/ros ↔ champs des messages ROS, sans ROS (testé sans ROS) ;
- `client` : connexion aiohttp, reconnexion, envoi des consignes (sans rclpy) ;
- `pont` : nœud rclpy.
"""
