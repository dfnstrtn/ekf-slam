use std::collections::HashMap;
/*
#[test]
fn slam_test_model() {
    
    let base_length = 20.0;
    let wheel_radius = 10.0;
    let odom_l = 20.0;
    let odom_r = 21.1;
        
    let test_observation= crate::ekf_slam::SensorDataType{
            r:20.0,
            phi:30.0,
            s:40.0
    }; 


    let mut slam = crate::ekf_slam::EKFSlam::new(10,None);
    let features= slam.feature_size;
    let mut sensor_error_covariance_matrix= na::DMatrix::<f32>::zeros(features, features);
    sensor_error_covariance_matrix[(0,0)] = 20.0;   
    sensor_error_covariance_matrix[(1,1)] = 10.0;   
    sensor_error_covariance_matrix[(2,2)] = 5.0;   
    slam.set_sensor_error_covariance_matrix(sensor_error_covariance_matrix);



    let mut motion_error_covariance_matrix= na::DMatrix::<f32>::zeros(features, features);
    motion_error_covariance_matrix[(0,0)] = 20.0;   
    motion_error_covariance_matrix[(1,1)] = 10.0;   
    motion_error_covariance_matrix[(2,2)] = 5.0;   
    slam.set_motion_error_covariance_matrix(motion_error_covariance_matrix);





    let mut motion_model = motion_models::odometry_motion_model::OdometryModel::new(base_length);
    
    let g_t = slam.get_dim_corrected_motion_linearizing_matrix(&mut motion_model,odom_l*wheel_radius,odom_r*wheel_radius);
    slam.motion_update(&mut motion_model,odom_l,odom_r);
    
    let r_t = slam.get_dim_corrected_motion_error_matrix();
    let mut sigma_bar = slam.find_linearized_model_covariance_matrix(g_t,r_t);
    let (mut h_matrix, mut estimated_observation) = slam.sensor_update_observation(&test_observation,9);
    
    let mut kalman_gain = match slam.get_kalman_gain(&mut sigma_bar,&mut h_matrix){
        Ok(g)=>g,
        Err(e)=>{panic!(e)}
    };

    slam.apply_mean_kalman_gain(&kalman_gain,
                              &estimated_observation
                              ,&test_observation);
    //slam.motion_update();
    slam.display_mean_matrix();
    slam.apply_covariance_kalman_gain(
                                    kalman_gain,
                                    h_matrix,
                                    sigma_bar);       
    }
    */
