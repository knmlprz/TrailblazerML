The issue with the task:
 - it doesn't as intended (discovered lately during doing task)
 - service_example.py starting service server for all tasks, but it should start services for starting robot
 - each node is the client instead of service server
 - service executor should be client node for all services
