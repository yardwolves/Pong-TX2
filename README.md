# Pong-TX2
Using Nvidia Jetson TX2 to interface with Atari Flashback, visually watch and play Pong game.  

The program was developed over Winter Term 2018 (Jan. - Mar.) at Oregon Tech - Portland Metro (Wilsonville, OR) for Embedded Systems II, EE 555. It is predominantly in c++ code, using OpenCV 3.4 libraries (later versions of OpenCV were missing specific functionality we needed), and Nvidia Cuda core GPU thread processing. 

The goals for the project are to use the Jetson TX2 camera to pull a live video stream of a screen, monitor, or projection of the game. Then to find the perspective angle it is viewing the skewed rectangular video at, by using color filter thresholding and a Harris Corner finding algorithm to find the image corners, and do a perspective transform to warp the video to a rectangular image. Color filter imaging is also used with a specified region of interest to find projected pong paddle and map its location using image contours. A region of interest of the pong playing field when the ball is in play (between the paddles) and a difference of images is used with contours to find the balls location per frame rate. The ball location is loaded into a vector point array with n-values(usually used with 8 or 10 points) within a least squares fit class. A least squares fit line function determines the immediate vector point slope, and an algorithm based on the boundaries of the pong field solve its intended destination for the paddle to volley ball back. A communication protocal for USB to FTDI communicates to a microcontroller running an Arduino based program, which tells which directional signal the microcontroller needs to send to the controller port (DB9 connectors) of the Atari Flashback console.

OpenCV Cuda functions are used when ever possible within the limitations of my ability to correctly access and use them.

The end goal was not fully accomplished. There are issues with the directional communication to console, and problems with live video perspective transform (works for imported recorded video). I doubt that I will be able to finish working out program bugs as this is a term long class project that has now ended and will not have access to another Jetson TX2.

