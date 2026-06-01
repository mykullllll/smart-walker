# Overview
The current smart walker design uses force torque sensors to measure intent of the user during walking. For the smart walker design that is used to help rehabilitate patients suffering from dementia, having a control system that only looks at the force being applied to the handles isn’t an accurate depiction of the user's intent since it’s not taking into account the users legs. In order to fix this problem with relatively cheap components, we’ve added a feed forward + feedback control system using a 2D RPLidar A1M8-R6 to perceive the users legs and an AK-10-9 V2.0 motor with magnetic encoders. 

![Hybrid Feedforward Feedback Control Loop](Docs/AFO_Control.drawio.svg)

# Functions

### Clustering & Filtering
* `scan_callback`: Returns LaserScan msg 
* `process_scan`: Returns x,y coordinates of laserscan values. 
* `cluster_find`: Identifies and returns centroids of clusters found.
* `kalman`: Predicts next centroid values based off of `cluster_find`
