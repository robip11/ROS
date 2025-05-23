# Turtlesim Fraktál Rajzoló (feleves)

Ez a ROS 2 csomag egy egyszerű arányos szabályozóval vezérli a [Turtlesim](http://wiki.ros.org/turtlesim) teknőst, hogy egy **Koch-pelyhet** rajzoljon. A program a ROS 2 kommunikációs mechanizmusait (topicok és szolgáltatások) használja a teknős mozgásának és tollának irányítására.

---

## Jellemzők

* **Arányos Szabályozó:** Egy alapvető P-szabályozót valósít meg a lineáris és szögsebességek vezérlésére, pontos célkövetést biztosítva.
* **Fraktál Rajzolás:** Rekurzívan generálja és rajzolja meg a Koch-pehely (Koch Snowflake) alakzatot.
* **Tollvezérlés:** A `turtlesim/srv/SetPen` ROS 2 szolgáltatás segítségével szabályozza a teknős tollát (toll fel/le, szín, vastagság).
* **Teleportálási Funkció:** A `turtlesim/srv/TeleportAbsolute` szolgáltatás segítségével a teknős azonnal a kívánt kezdő pozícióba ugrik.
* **ROS 2 Kompatibilitás:** Teljesen ROS 2-kompatibilis, Pythonban (rclpy) íródott node-ként, topicok és szolgáltatások felhasználásával.

---

## Első lépések

Ezek az utasítások segítenek a projekt helyi gépen történő futtatásában.

### Előfeltételek

* Telepített és beállított [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html) (vagy újabb) disztribúció.
* A `turtlesim` ROS 2 csomag telepítve van:
    ```bash
    sudo apt install ros-humble-turtlesim # vagy a disztribúciódnak megfelelő csomag
    ```

### Telepítés

1.  **Hozd létre a ROS 2 munkaterületedet (ha még nincs):**
    ```bash
    mkdir -p ~/feleves_ws/src
    cd ~/feleves_ws/
    ```

2.  **Klónozd a tárolót:**
    Navigálj a `src` mappába, és klónozd ide a GitHub tárolódat.
    ```bash
    cd ~/feleves_ws/src
    git clone [https://github.com/robip11/ROS.git](https://github.com/robip11/ROS.git) # Vagy a saját tárolód URL-je
    ```
    *Győződj meg róla, hogy a `feleves` csomag a klónozott tárolón belül van.*

3.  **Fordítsd le a csomagot:**
    Lépj vissza a munkaterületed gyökerébe, és fordítsd le a csomagot a `colcon` segítségével.
    ```bash
    cd ~/feleves_ws/
    colcon build
    ```

4.  **Forrásold a beállításokat:**
    Ez a lépés elengedhetetlen, hogy a ROS 2 megtalálja az újonnan lefordított csomagodat. Ezt minden **új terminálban** meg kell tenned!
    ```bash
    source install/setup.bash
    ```

---

## Használat

A kontroller elindításához és a teknős mozgásának megtekintéséhez futtasd a launch fájlt:

```bash
ros2 launch feleves start_controller.launch.py
