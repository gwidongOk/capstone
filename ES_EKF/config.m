function C = config()
%CONFIG  ES-EKF tuning and sensor baseline values.

    C.dt_imu  = 1/416;
    C.dt_gps  = 1/25;
    C.dt_baro = 1/50;
    C.dt_mag  = 1/50;

    C.var_acc_datasheet_psd  = (220e-6 * 9.80665)^2;
    C.var_gyro_datasheet_psd = (3.8e-3 * pi / 180)^2;
    C.var_acc_datasheet      = C.var_acc_datasheet_psd / C.dt_imu;
    C.var_gyro_datasheet     = C.var_gyro_datasheet_psd / C.dt_imu;

    C.var_acc_static         = 2.729445e-4;
    C.var_gyro_static        = 8.061781e-7;

    C.var_acc                = max(C.var_acc_datasheet, C.var_acc_static);
    C.var_gyro               = max(C.var_gyro_datasheet, C.var_gyro_static);
    C.var_ba             = (0.001)^2;
    C.var_bg             = (0.0001)^2;

    C.var_gps_pos_h_datasheet = (2.0 / 1.1774)^2;
    C.var_gps_pos_v_datasheet = (2.0 * 2.0 / 1.1774)^2;
    C.var_gps_pos_h_static    = 0.694^2;
    C.var_gps_pos_v_static    = 1.282927^2;
    C.var_gps_pos_h           = max(C.var_gps_pos_h_datasheet, C.var_gps_pos_h_static);
    C.var_gps_pos_v           = max(C.var_gps_pos_v_datasheet, C.var_gps_pos_v_static);

    C.var_gps_vel_h_datasheet = 0.05^2;
    C.var_gps_vel_v_datasheet = 0.05^2;
    C.var_gps_vel_h_static    = 0.2684742^2;
    C.var_gps_vel_v_static    = 0.2684742^2;
    C.var_gps_vel_h           = max(C.var_gps_vel_h_datasheet, C.var_gps_vel_h_static);
    C.var_gps_vel_v           = max(C.var_gps_vel_v_datasheet, C.var_gps_vel_v_static);

    C.var_baro_datasheet = 0.11^2;
    C.var_baro_static    = 4.920342e-2;
    C.var_baro           = max(C.var_baro_datasheet, C.var_baro_static);

    C.var_mag_datasheet = (0.4 / 500)^2;
    C.var_mag_static    = (1.963 / 500)^2;
    C.var_mag           = max(C.var_mag_datasheet, C.var_mag_static);

    C.g_ned     = [0; 0; 9.81];
    C.m_ref_ned = [0.5961; -0.0838; 0.7986];

    C.P0_pos = 3.0;
    C.P0_vel = 5.0;
    C.P0_att = 0.087;
    C.P0_ba  = 0.5;
    C.P0_bg  = 0.01;

    % C.Q_ACC_BASE_SCALE  = 7.0;
    % C.Q_GYRO_BASE_SCALE = 3.0;
    % C.GPS_POS_R_SCALE   = 1.2;
    % C.GPS_VEL_R_SCALE   = 1.2;
    % C.GPS_ACC_INFLATION = 3.0;
    % C.BARO_R_SCALE      = 1.5;

    C.Q_ACC_BASE_SCALE  = 1000.0;
    C.Q_GYRO_BASE_SCALE = 10.0;
    C.GPS_POS_R_SCALE   = 1.0;
    C.GPS_VEL_R_SCALE   = 1.0;
    C.GPS_ACC_INFLATION = 3.0;
    C.BARO_R_SCALE      = 1.0;

    C.JERK_ALPHA     = 0.02;
    C.JERK_THRESH    = 120.0;
    C.ANG_ACC_THRESH = 8.0;
    C.JERK_SCALE_MAX = 12.0;
end
