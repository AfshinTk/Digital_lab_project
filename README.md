# Digital_lab_project
### Afshin Tavakolikia 9823022
***
The attacker robot attemp to reach the ball and make it goal by hitting toward opponent's gate and the defender(opponent) attemp to protect the gate.

## Astrategy & Algorithms
### Attacker's astrategy
The attacker first tries to look for the ball, if he cannot find the ball's location, he waits for the ball to approach him or for the goalkeeper to receive the coordinates of the ball relative to the goalkeeper and the goalkeeper's coordinates. After finding the ball, he tries to use the distance and angle he has with the ball as an error and goes towards the ball with the controller. After that, he tries to bring his angle and the ball closer to the opponent's goal and shoots towards it.
The attacker chasing ball with `class ChasingBall` and hit the ball continuously until make a goal.

***
### Defender's astrategy
at the beginning, the defender goes to the gate. if the ball come to the near of the gate(approximately in penalty area), the defender tries to hit the ball to make it go away and protect the gate.
