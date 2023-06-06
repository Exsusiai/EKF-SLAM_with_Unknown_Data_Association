# EKF-SLAM with Unknown Data Association
 This project is the Final project of Course [EL2320 Applied Estimation](https://www.kth.se/kurs-pm/EL2320/EL232020232-50443) at KTH Royal Institute of Technology

## Description
This report mainly discusses the specific implementation of the EKF-SLAM algorithm in the case of unknown data association, which is observation order of the landmark is unknown, and we will use normalized Mahalanobis distance to judge whether the observed landmark is new or old.
The conclusion of this report is that when we do not know the data association, the stability of the algorithm is not strong, and the use of Mahalanobis distance to identify landmarks may fail, so the basic algorithm may not be applicable to more complex environments.

## Usage
### 1. Install Matlab
You can download and Install Matlab at their [official website](www.mathworks.com/products/). Please make sure its version is **R2021b** or higher.
### 2. Clone this project
```
git clone https://github.com/Exsusiai/EKF-SLAM_with_Unknown_Data_Association.git
```
### 3. Run the main project file
```
cd Project_code
```
**Run "ekf_slam.m"**

## Running result
### results display

### Report
To see the report, please check [Project_Report.pdf](./Project_Report.pdf)


## Author

[Jingsheng Chen](mailto:chjingsheng@gmail.com)  @KTH Royal Institute of Technology


