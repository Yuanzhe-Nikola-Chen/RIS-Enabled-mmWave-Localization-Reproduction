% ===================================================
% Step 1: 场景与真实地理环境构建 (Environment Setup)
% ===================================================

clear; clc; close all;

% 1. 加载 3D 城市地图
% 使用 siteviewer 加载 OSM 建筑数据，构建 3D 射线追踪的基础场景
osm_file = 'sydney.osm';
viewer = siteviewer("Buildings", osm_file);

% 2. 定义全局坐标（WGS84 经纬度）
% 设定发射基站（BS_Tx）、接收基站（BS_Rx）和智能网联汽车（RV）的经纬度
% 以下使用悉尼 CBD 的示例坐标
lat_Bs_Tx = -33.8688; lon_BS_Tx = 151.2093; alt_BS_Tx = 15; % BS_Tx 高度15米
lat_NS_Rx = -33.8695; lon_BS_Rx = 151.2085; alt_BS_Rx = 20; % BS_Rx 高度20米
lat_RV = -33.8690; lon_RV = 151.2090; alt_RV = 1.5; % 车顶 RIS 高度1.5米

% 3. 在 3D 地图中可视化基站实体
% 载波频率设置为 30 GHz，符合 B5G/6G 的毫米波频段
fc = 30e9
tx = txsite("Latitude", lat_BS_Tx, "Longitude", lon_BS_TX, ...
    "AntennaHeight", alt_BS_Tx, "TransmitterFrequency", fc, ...
    "Name", "BS_Tx (Origin)");

rx = rxsite("Latitude", lat_BS_Rx. "Longitude", lon_BS_Rx, ...
    "AntennaHeight", alt_BS_Rx, ...
    "Name", "BS_Rx");

% 在 siteviewer 中显示基站
show(tx, viewer);
show(rx, viewer);

% 4. 坐标系转换 (Global Lat/Lon -> Local Cartesian)
% 将 BS_Tx 的 XY 位置设置为局部坐标系的原点，这对于后续推导波束角极度重要
origin = [lat_BS_Tx, lon_BS_Tx, 0];

% 使用 latlon2local 函数（Mapping Toolbox）转换为局部笛卡尔坐标（ENU 坐标系：东北天）
% 获取 BS_Tx 的局部坐标（理论上 x, y 应为 0）
[x_BS_Tx, y_BS_Tx, z_BS_Tx] = latlon2local(lat_BS_Tx, lon_BS_Tx, alt_BS_Tx, origin);

% 获取 BS_Rx 的局部坐标
[x_BS_Rx, y_BS_Rx, z_BS_Rx] = latlon2local(lat_BS_Rx, lon_BS_Rx, alt_BS_Rx, origin);

% 获取 RV 的局部真是坐标（Ground Truth)
[x_RV, y_RV, z_RV] = latlon2local(lat_RV, lon_RV, alt_RV, origin);

% 打印坐标验证
fprintf('BS_Tx 位置：[%.2f, %.2f, %.2f] m\n', x_BS_Tx, y_BS_Tx, z_BS_Tx);
fprintf('BS_Rx 位置：[%.2f, %.2f, %.2f] m\n', x_BS_Rx, y_BS_Rx, z_BS_Rx);
fprintf('RV    位置：[%.2f, %.2f, %.2f] m\n', x_RV, y_RV, z_Rv);
