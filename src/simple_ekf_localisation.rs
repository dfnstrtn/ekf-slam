use crate::simple_ekf_slam;
use simple_ekf_slam::{con_bear,sin,cos};
use simple_ekf_slam::{Float,Matrix,mX,mY,mT,jX,jY,jT};
use simple_ekf_slam::{mat_add,mat_mul,mat_sub,print_matrix};

pub fn atan2(y:Float,x:Float)->Float{
    y.atan2(x)
}

pub fn mat_mul3(mat_left:&Matrix,mat_middle:&Matrix,mat_right:&Matrix)->Matrix{
    mat_mul( mat_left , &mat_mul(mat_middle,&mat_right))
}




pub struct EkfParams{
    pub num_obsts :usize,
    pub num_params_movt:usize,
    pub num_params_obst:usize,
    pub num_elements:usize,
    pub delta_T:f32,
    pub alpha:(f32,f32,f32,f32)
}
impl EkfParams{
    pub fn new(num_params_movt:usize,num_params_obst:usize,num_obsts:usize,delta_T:f32)->Self{
        Self{
            num_obsts,
            num_params_movt,
            num_params_obst,
            num_elements : num_obsts*num_params_obst + num_params_movt,
            delta_T,
            alpha:(0.,0.,0.,0.)
        }
    }
}





pub fn odom_get_v_w(odom_prev:(Float,Float) ,odom_now:(Float,Float) , base_length:Float, wheel_radius:Float)->(Float,Float){
    let diff = ( (odom_now.0-odom_prev.0)*wheel_radius ,  (odom_now.1-odom_prev.1)*wheel_radius );
    let alpha = (diff.1-diff.0)/base_length;
    let delta_s = (diff.1 + diff.0)/2.;
    (delta_s,alpha)
}


pub fn vel_motion_update(v:f32,w:f32,delta_T:f32,Theta:f32)->Matrix{
    let mut vel_update = Matrix::zeros(3,1);
    if w==0.{
        vel_update[mX] = (v)*cos(Theta)*delta_T;
        vel_update[mY] = (v)*sin(Theta)*delta_T;
        vel_update[mT] = 0.;
    }else{
        vel_update[mX] = (-v/w)*sin(Theta) + (v/w)*sin(Theta + w*delta_T);
        vel_update[mY] = (v/w)*cos(Theta) - (v/w)*cos(Theta + w*delta_T);
        vel_update[mT] = con_bear(w*delta_T); 
    }
    vel_update
}



pub fn vel_jacobian_update(v:f32,w:f32,delta_T:f32,Theta:f32)->Matrix{
    let mut jacobian_update = Matrix::zeros(3,3);    
    if w==0.{
        jacobian_update[jX] = -(v)*sin(Theta);
        jacobian_update[jY] = (v)*cos(Theta);
        jacobian_update[jT] = 0.;
    }else{
        jacobian_update[jX] = (-v/w)*cos(Theta) + (v/w)*cos(Theta + w*delta_T);
        jacobian_update[jY] = (-v/w)*sin(Theta) + (v/w)*sin(Theta + w*delta_T);
        jacobian_update[jT] = 0.;
    }
    jacobian_update
}



pub fn get_rt(a:(f32,f32,f32,f32),v:f32,w:f32,Theta:f32,delta_T:f32)->Matrix{ 
    let mut Mt = Matrix::zeros(2,2);
    Mt[(0,0)] =  a.0*v.powi(2) + a.1*w.powi(2);
    Mt[(1,1)] =  a.2*v.powi(2) + a.3*w.powi(2);
    let mut Vt = Matrix::zeros(3,2);
    if w==0.0{     
        Vt[(0,0)] = cos(Theta)*delta_T;
        Vt[(1,0)] = sin(Theta)*delta_T;
        Vt[(2,1)] = 0.0;
    }else{    
        let sint = Theta.sin();
        let cost = Theta.cos();
        let sint_wt = (Theta+w*delta_T).sin();
        let cost_wt = (Theta+w*delta_T).cos();

        Vt[(0,0)] =  (-sint + sint_wt)/w;
        Vt[(0,1)] =  v*(sint - sint_wt)/(w*w)  + v*cost_wt/w;
        Vt[(1,0)] = (cost - cost_wt)/w;
        Vt[(1,1)] = -(v*(cost - cost_wt))/(w*w) + v*sint_wt/w;
        Vt[(2,0)] = 0.;
        Vt[(2,1)] = 1.;
    }

    mat_mul(&Vt,&mat_mul(&Mt,&Vt.transpose()))
}





#[allow(non_snake_case,unused)]
pub fn ekf_localisation_step(
            params:&mut EkfParams,
            R:&mut Matrix,
            Q:&mut Matrix,
            mean_prev:&mut Matrix,
            cov_prev:&mut Matrix,
            cmd_vel:&mut Matrix,
            obs_vec:&mut Vec<(Matrix,usize)>,
            map_vec:&Vec<Matrix>,
            did_see:&mut Vec<bool>
            ){
        
    let I = Matrix::identity(3,3);
    let delta_T = params.delta_T; 
    let v = cmd_vel[mX];
    let w = cmd_vel[mY];
    let T_prev = mean_prev[mT];
    let vel_update = vel_motion_update(v,w, delta_T ,T_prev);
    
    let mut mean_t = mat_add(&mean_prev,&vel_update); 

    // maybe do R update function here 
    //jacobian update  -O
    let mtest = 0.001;
    let Rt =  get_rt((mtest,mtest,mtest,mtest),v,w,T_prev,delta_T);

    let mut jacobian_update =  vel_jacobian_update(v,w,delta_T,T_prev);
    let Gt = mat_add(&I , &jacobian_update);
    let mut cov_t = mat_add( &mat_mul(&Gt,&mat_mul(&cov_prev,&Gt.transpose())), &Rt);
   
    //obs_vec.clear();
    obs_vec.iter().map(|obs |{
        println!("observation made");
        let index = obs.1;
        let Theta_t = mean_t[mT];
        let deltaX = map_vec[index][mX] - mean_t[mX];  
        let deltaY =  map_vec[index][mY] - mean_t[mY];
        
        let q = deltaX.powi(2) + deltaY.powi(2);
        let q_r = q.sqrt();

        let mut z_t = Matrix::zeros(2,1);
        z_t[mX] = q.sqrt();
        z_t[mY] = con_bear(atan2(deltaY,deltaX) - Theta_t);

        let mut H_i = Matrix::zeros(2,3);
        H_i[(0,0)] =  - deltaX /q_r;
        H_i[(0,1)] =  - deltaY /q_r;
        H_i[(0,2)] =  0.0;
        H_i[(1,0)] =  deltaY /q;
        H_i[(1,1)] =  - deltaX /q;
        H_i[(1,2)] =  - 1.;
        
        let Si = mat_add( &mat_mul3(&H_i,&cov_t,&H_i.transpose()) , &Q) ;
        

        let Ki = mat_mul3(&cov_t,&H_i.transpose(),&Si.try_inverse().unwrap());
        print!("mean before");
        print_matrix(&mean_t);
        mean_t = mat_add( &mean_t ,  &mat_mul(&Ki,&mat_sub(&obs.0,&z_t)) ) ;
        
        print!("mean after");
        print_matrix(&mean_t);
        cov_t = mat_mul(&mat_sub(&I,&mat_mul(&Ki,&H_i)), &cov_t); 
    }).count();
    *mean_prev = mean_t;
    *cov_prev = cov_t;
}    


#[allow(unused)]
#[cfg(test)]
mod test{
    use crate::simple_ekf_slam;    
    use simple_ekf_slam::{con_bear,sin,cos};
    use simple_ekf_slam::{Float,Matrix,mX,mY,mT,jX,jY,jT};
    use simple_ekf_slam::odom_get_v_w;
    use probability::prelude::*;


    fn simulate_test_objects(x:f32,y:f32,theta:f32,index:usize) -> Vec<(Matrix,usize)>{
        //adding gaussian noise
        let mut source = source::default();
        let distribution = Gaussian::new(0.,0.04);
        let mut sampler = Independent(&distribution,&mut source);

        let objs = vec![(2.5,2.5),(-2.5,2.5),(-2.5,-2.5),(2.5,-2.5) ];

        objs.iter().enumerate().filter(|m|{ ((m.1.0-x).powi(2) +  (m.1.1-y).powi(2)) < 4.0}).map(|m|{
            let mut z = Matrix::zeros(2,1);
            let x_o = m.1.0;
            let y_o = m.1.1;

            let r = ((x-x_o).powi(2) + (y-y_o).powi(2)).sqrt().abs(); 
            let r_sampled = r + sampler.next().unwrap() as f32;
            
            let phi =  Float::atan2( y_o-y , x_o-x )  - theta ; 
            let phi_sampled = phi + sampler.next().unwrap() as f32;
            
            z[mX] = r_sampled;              
            z[mY] = con_bear(phi_sampled);
            (z,m.0)
        }).collect::<Vec<(Matrix,usize)>>() 
    }
    



    use crate::simple_ekf_slam::tests::file_read_odom_accurate;
    use crate::simple_ekf_localisation::{atan2,EkfParams,ekf_localisation_step};
    use std::io::Write;
    static BASE_LENGTH:Float = 0.1054;
    static WHEEL_RADIUS:Float = 0.021;

    #[test]
    fn simple_ekf_localisation_test(){
        let mut mean = Matrix::zeros(3,1);
        mean[mT] = 1.57; 

        let mut cov = Matrix::zeros(3,3);
        let mut cmd_vel = Matrix::zeros(2,1);
        let mut params = EkfParams::new(3,3,4,1.0);
        
        
        // Motion model error matrix 
        let mut R = Matrix::identity(3,3);
        R[(0,0)] = 0.001;
        R[(1,1)] = 0.001;
        R[(2,2)] = 0.001;
        // TODO (jfarhan) : pass alphas as a parameter instead of this useless R


        // Sensor model error matrix
        let mut Q = Matrix::identity(2,2);
        Q[(0,0)] = 0.01;
        Q[(1,1)] = 0.01;


        let odom_data = file_read_odom_accurate( std::path::Path::new("raw_data/ws_pos8008.txt"),std::path::Path::new("raw_data/abs_pos8008.txt")).unwrap();         
        
        
        let mut wf = std::fs::File::create("raw_data/ekf_localisation_output.txt").unwrap();
        let mut writer = std::io::BufWriter::new(&mut wf);
        
        let mut odom_prev = (0.,0.);
        let mut abs_prev = (0.,0.);


        
        
        let map_objs = vec![(2.5,2.5),(-2.5,2.5),(-2.5,-2.5),(2.5,-2.5) ];
        let map_vec =map_objs.into_iter().map(|m|{
            let mut p = Matrix::zeros(3,1);
            p[mX] = m.0;
            p[mY] = m.1;
            p
        }).collect::<Vec<Matrix>>();
        

        map_vec.iter().map(|m|{
            println!("{:?}",m);
        }).count();

        

        odom_data.iter().enumerate().map(|(index,(odom,abs))|{
            let odom_new = odom; 
            let x_abs = abs.0;
            let y_abs = -abs.1;
            let angle_abs = con_bear(atan2(y_abs - abs_prev.1,x_abs - abs_prev.0));
            abs_prev = (x_abs,y_abs);
            
            let v_w = odom_get_v_w(odom_prev,*odom,BASE_LENGTH,WHEEL_RADIUS);
            cmd_vel[mX] = v_w.0;
            cmd_vel[mY] = v_w.1;

            odom_prev.0 = odom.0;
            odom_prev.1 = odom.1;

            let mut obs_vec = simulate_test_objects(x_abs,y_abs,angle_abs,index);
            
            println!("[{}]============",index);

            ekf_localisation_step(
                &mut params,
                &mut R,
                &mut Q,
                &mut mean,
                &mut cov,
                &mut cmd_vel,
                &mut obs_vec,
                &map_vec,
                &mut Vec::new()
                );
        }).count();

    }
}

