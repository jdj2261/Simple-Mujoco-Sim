# Introduction to Humanoid Robotics

Shuuji Kajita · Hirohisa Hirukawa · Kensuke Harada · Kazuhito Yokoi

[TOC]

## Numerical Solution to Inverse Kinematics

by **Newton-Raphson method** 

출처  : [Wikipedia](https://en.wikipedia.org/wiki/Newton%27s_method)

<img src="../img/newton.gif"  height="400">



<img src= "../img/robotics-03_img1.png" height="300" />

**Algorithm**

- Step 1. Prepare the position and attitude (<a href="https://www.codecogs.com/eqnedit.php?latex=p^{ref},&space;R^{ref}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?p^{ref},&space;R^{ref}" title="p^{ref}, R^{ref}" /></a>) of the base link

- Step 2. Prepare the position and attitude (<a href="https://www.codecogs.com/eqnedit.php?latex=p^{ref},&space;R^{ref}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?p^{ref},&space;R^{ref}" title="p^{ref}, R^{ref}" /></a>) of the target link

- Step 3. Define vector **q** which holds the joint angles from the base link to the target link

- Step 4. Use forward kinematics to calculate the position and attitude(<a href="https://www.codecogs.com/eqnedit.php?latex=p,&space;R" target="_blank"><img src="https://latex.codecogs.com/gif.latex?p,&space;R" title="p, R" /></a>) of the target link

- Step 5. Calculate the difference in position and attitude (<a href="https://www.codecogs.com/eqnedit.php?latex=\Delta&space;p,&space;\Delta&space;R" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\Delta&space;p,&space;\Delta&space;R" title="\Delta p, \Delta R" /></a>) = (<a href="https://www.codecogs.com/eqnedit.php?latex=p^{ref}&space;-&space;p,&space;R^{T}R^{ref}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?p^{ref}&space;-&space;p,&space;R^{T}R^{ref}" title="p^{ref} - p, R^{T}R^{ref}" /></a>)

- Step 6. If (<a href="https://www.codecogs.com/eqnedit.php?latex=\Delta&space;p,&space;\Delta&space;R" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\Delta&space;p,&space;\Delta&space;R" title="\Delta p, \Delta R" /></a>) are small enough, stop the calculation
- **Step 7**. If (<a href="https://www.codecogs.com/eqnedit.php?latex=\Delta&space;p,&space;\Delta&space;R" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\Delta&space;p,&space;\Delta&space;R" title="\Delta p, \Delta R" /></a>) are not small enough, calculate <a href="https://www.codecogs.com/eqnedit.php?latex=\Delta{q}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\Delta{q}" title="\Delta{q}" /></a> which would reduce the error
- **Step 8**. Update joint angles by <a href="https://www.codecogs.com/eqnedit.php?latex=q&space;:=&space;q&space;&plus;&space;\Delta{q}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?q&space;:=&space;q&space;&plus;&space;\Delta{q}" title="q := q + \Delta{q}" /></a> and return to **Step 4**



**Two Questions**

1. What do we really mean by the position and attitude errors (<a href="https://www.codecogs.com/eqnedit.php?latex=\Delta&space;p,&space;\Delta&space;R" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\Delta&space;p,&space;\Delta&space;R" title="\Delta p, \Delta R" /></a>) being small enough? (**Step 7**)

   - <a href="https://www.codecogs.com/eqnedit.php?latex=\Delta{p}&space;=&space;0,&space;\Delta{R}&space;=&space;E" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\Delta{p}&space;=&space;0,&space;\Delta{R}&space;=&space;E" title="\Delta{p} = 0, \Delta{R} = E" /></a>

     <a href="https://www.codecogs.com/eqnedit.php?latex=err(\Delta{p},&space;\Delta{R})&space;=&space;\left&space;\|&space;\Delta{p}&space;\right&space;\|^{2}&space;&plus;&space;\left&space;\|&space;\Delta{\theta}&space;\right&space;\|^{2},&space;\Delta{\theta}&space;\equiv&space;(ln\Delta{R})^{\vee}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?err(\Delta{p},&space;\Delta{R})&space;=&space;\left&space;\|&space;\Delta{p}&space;\right&space;\|^{2}&space;&plus;&space;\left&space;\|&space;\Delta{\theta}&space;\right&space;\|^{2},&space;\Delta{\theta}&space;\equiv&space;(ln\Delta{R})^{\vee}" title="err(\Delta{p}, \Delta{R}) = \left \| \Delta{p} \right \|^{2} + \left \| \Delta{\theta} \right \|^{2}, \Delta{\theta} \equiv (ln\Delta{R})^{\vee}" /></a>

2. How do we actually go about calculating <a href="https://www.codecogs.com/eqnedit.php?latex=\Delta{q}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\Delta{q}" title="\Delta{q}" /></a>, to narrow the gap? (**Step 8**)
