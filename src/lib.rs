//pub mod differential_drive;
pub extern crate nalgebra as na;


#[allow(non_snake_case,non_camel_case_types)]
pub mod ekf_slam;

#[allow(non_snake_case,non_camel_case_types)]
pub mod simple_ekf_slam;


#[allow(non_snake_case,non_camel_case_types)]
pub mod simple_pf_slam;



#[allow(non_snake_case,non_camel_case_types)]
pub mod simple_ekf_localisation;


#[cfg(test)]
pub mod tests;

pub fn normalize_angle(angle:f32)->f32{
    if angle>std::f32::consts::PI{
        return -(2.*std::f32::consts::PI - angle);
    }else if angle< -1.*std::f32::consts::PI{
        return 2.*std::f32::consts::PI + angle;
    }else{
        return angle;
    }
}

