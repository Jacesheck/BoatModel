# Boat simulator

Feel free to simulate differentially driven boat.

## Contributing to the project:

1. Install [git](https://git-scm.com/downloads)
2. Install [python](https://www.python.org/downloads/) **Add to PATH**
3. Open cmd prompt or powershell and type the following commands

```
pip install numpy
pip install pygame
pip install bleak
pip install matplotlib
```

4. Clone project into file of choice, example

```
cd C:/Users/{username}/Desktop/python/boat
git clone https://github.com/Jacesheck/BoatModel.git
cd BoatModel

```

5. Open `boat.py` in editor of choice
6. Checkout on another branch

```
git checkout -b nameOfNewBranch
```

6. Change some code
7. Run/Test code with `python boat.py`
8. When you are happy with your code commit it (pull to check any updates)

```
git add boat.py
git commit
```

9. Write a message
10. Push to cloud

```
git push
```

11. Create pull request (or ask me)

### Boat state space model

$$
x=
\left(\begin{array}{cc}
x\\
y\\
\dot{x}\\
\dot{y}\\
\theta\\
\dot{\theta}
\end{array}\right)
$$

$$
F=
\left(\begin{array}{cc}
1 & 0 & dt & 0 & 0 & 0\\
0 & 1 & 0 & dt & 0 & 0\\
0 & 0 & 1-dt\cdot b_1 & 0 & 0 & 0\\
0 & 0 & 0 & 1-dt\cdot b_1 & 0 &  0\\
0 & 0 & 0 & 0 & 1 & dt\\
0 & 0 & 0 & 0 & 0 & 1-dt\cdot b_2
\end{array}\right)
$$

$$
B=
\left(\begin{array}{cc}
0 & 0\\
0 & 0\\
dt\cdot cos(\theta) & dt\cdot cos(\theta)\\
dt\cdot sin(\theta) & dt\cdot sin(\theta)\\
0 & 0\\
dt\cdot w\over2 & -dt\cdot w\over2\\
\end{array}\right)
$$

$dt$ - Delta time\
$\theta$ - Heading\
$w$ - Width of boat\
$b_1$ - Velocity drag coefficient\
$b_2$ - Rotational drag coefficient
