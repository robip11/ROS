Turtlesim Fraktál Rajzoló (feleves)
Ez a ROS 2 csomag a Turtlesim szimulátorban egy arányos szabályozó segítségével Koch-pelyhet rajzol. A teknős előre meghatározott rekurzív módon generált útpontokat követ, és a toll fel-le mozgatásával hozza létre a fraktálmintát.

Működés
A program a feleves/feleves/controller.py fájlban található ProportionalController node-ot használja. Ez a node:

Előfizet a teknős aktuális pozíciójára (/turtle1/pose).
Közzétesz sebességparancsokat (/turtle1/cmd_vel).
Használja a set_pen és teleport_absolute szolgáltatásokat.
A fraktál rajzolását a koch_curve rekurzív függvény kezeli, amelyet a draw_koch_snowflake függvény hív meg háromszor, a Koch-pehely három oldalához.

Futtatás
Fordítás: Ha módosítottad a kódot, fordítsd újra a munkaterületet:

cd ~/feleves_ws/

colcon build
Forrásolás: Minden új terminálban, vagy a fordítás után:

source install/setup.bash

Indítás: Futtasd a launch fájlt:

ros2 launch feleves start_controller.launch.py

A Turtlesim ablakban megjelenik a teknős, és elkezdi rajzolni a Koch-pelyhet.

Paraméterezés
A feleves/feleves/controller.py fájl main függvényében módosíthatod a fraktál rajzolásának paramétereit:

fractal_order: A fraktál részletességi szintje (pl. 2 vagy 3). Minél nagyobb az érték, annál részletesebb (és lassabb) a rajz.
fractal_size: A fraktál alapjának (háromszög oldalainak) hossza.
start_pos_x, start_pos_y: A teknős kezdő pozíciója a rajzoláshoz.
