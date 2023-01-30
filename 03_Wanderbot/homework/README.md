# Homework 1 - Wanderbot

---

- This assignment is due on (**a date and time to be determined**)
- Email the following three (3) items to me:
    - A markdown file named `README.md` which provides answers to the questions below.
    - Your modified version of `echo_scan.py`, which you used to answer the questions from Part 1 below.
    - Your modified `wander.py` script, which you used to answer Part 2 below.
 
---

1. Take a look at the `echo_scan.py` Python script (located in this directory on GitHub).  This script provides you with some code to help get you started.  At the bottom of the script there are several questions.  Modify the Python code to answer those questions.  When you are asked to create a plot, you might find the following code helpful:

    ```
    import matplotlib.pyplot as plt
    
    x = [1, 3, 4, 7]
    y = [2, 5, 1, 6]
    
    plt.plot(x, y, '*')
    plt.axis([min(x)-1, max(x)+1, min(y)-1, max(y)+1])
    plt.show()
    ```

2. Modify the `wander.py` script to make your turtlebot do a better job of avoiding obstacles.  

3. BONUS:  Propose an effective approach for addressing the issue that the laser scanner cannot differentiate between obstacles that are closer than `range_min` or further away than `range_max`.  You approach can involve strategically moving your robot to help determine if an `nan` value really means that there's no obstacle there, or that the obstacle is closer than `range_min`.  You may also subscribe to the `/mobile_base/events/bumper` topic to detect if your robot has made contact with an obstacle.


