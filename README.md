# Boat simulator

Feel free to simulate differentially driven boat.\

## Contributing to the project:

1. Install [git](https://git-scm.com/downloads)
2. Install [python](https://www.python.org/downloads/)
3. Open cmd prompt or powershell and type the following commands

```
pip install numpy
pip install pygame
```

4. Clone project into file of choice, example

```
cd C:/Users/{username}/Desktop/python/boat
git clone ...
cd BoatGame

```

5. Open `boat.py` in editor of choice
6. Checkout on another branch

```
git checkout -b nameOfNewBranch
```

6. Change some code
7. When you are happy with your code commit it (pull to check any updates)

```
git add boat.py
git commit
```

8. Write a message
9. Push to cloud

```
git push
```

10. Create pull request (or ask me)

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
0 & 0 & 1-dt\cdot v\cdot b_1 & 0 & 0 & 0\\
0 & 0 & 0 & 1-dt\cdot v\cdot b_1 & 0 &  0\\
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
