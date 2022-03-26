use na::DMatrix;
pub type Matrix = DMatrix<f32>;
pub type Float = f32;
static _2PI :Float = std::f32::consts::PI * 2.0;
pub static mX:(usize,usize) = (0,0);
pub static mY:(usize,usize) = (1,0);
pub static mT:(usize,usize) = (2,0);
static mDS:(usize,usize) = (3,0);


pub static jX:(usize,usize) = (0,2);
pub static jY:(usize,usize) = (1,2);
pub static jT:(usize,usize) = (2,2);


static mR:(usize,usize) = (0,0);
static mPhi:(usize,usize) = (1,0);



pub fn sin(value:Float)->Float{
    value.sin()
}

pub fn cos(value:Float)->Float{
    value.cos()
}


pub fn con_bear(value:Float)->Float{
    if value>_2PI{
        value - _2PI
    }else{
        if value< -_2PI{
            return value+_2PI;
        }
        value
    }
}




pub fn mat_mul(mat_A:&Matrix,mat_B:&Matrix)->Matrix{
    let rows = mat_A.shape().0;
    let cols = mat_B.shape().1;
    let mut res = Matrix::zeros(rows,cols);
    mat_A.mul_to(mat_B,&mut res);
    res
}



pub fn mat_add(mat_A:&Matrix,mat_B:&Matrix)->Matrix{
    let rows = mat_A.shape().0;
    let cols = mat_B.shape().1;
    let mut res = Matrix::zeros(rows,cols);
    mat_A.add_to(mat_B,&mut res);
    res
}


//  this is probably a wee bit errorsome 
pub fn mat_sub(mat_A:&Matrix,mat_B:&Matrix)->Matrix{
    let rows = mat_A.shape().0;
    let cols = mat_B.shape().1;
    let mut res = Matrix::zeros(rows,cols);
    mat_A.sub_to(mat_B,&mut res);
    res
}


// I should add a string here probably with the matrix 
pub fn print_matrix(m:&Matrix){
    print!("\nMatrix :[{} x {}]\n",m.shape().0,m.shape().1);
    m.row_iter().for_each(|n|{
        print!("[");
        n.iter().for_each(|p|{
            print!("{} ",p);
        });
        print!("]\n");
    })
}









pub struct EkfSLAMParams{
    pub num_obsts :usize,
    pub num_params_movt:usize,
    pub num_params_obst:usize,
    pub num_elements:usize,
    pub delta_T:f32,
    pub alpha:(f32,f32,f32,f32)
}
impl EkfSLAMParams{
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












// https://in.mathworks.com/help/robotics/ug/mobile-robot-kinematics-equations.html#MobileRobotKinematicsEquationsExample-5 
pub fn vel_motion_update_matlab(v:f32,w:f32,delta_T:f32,Theta:f32)->Matrix{
    let mut vel_update = Matrix::zeros(3,1);
    vel_update[mX] = v*cos(Theta)*delta_T;
    vel_update[mY] = v*sin(Theta)*delta_T;
    vel_update[mT] = con_bear(w);
    vel_update
}


// https://in.mathworks.com/help/robotics/ug/mobile-robot-kinematics-equations.html#MobileRobotKinematicsEquationsExample-5 
pub fn vel_jacobian_update_matlab(v:f32,w:f32,delta_T:f32,Theta:f32)->Matrix{
    let mut vel_update = Matrix::zeros(3,3);
    vel_update[jX] = -v*sin(Theta)*delta_T;
    vel_update[jY] = v*cos(Theta)*delta_T;
    vel_update[jT] = 0.;
    vel_update
}



pub fn get_rt_matlab(a:(f32,f32,f32,f32),v:f32,w:f32,Theta:f32,delta_T:f32)->Matrix{ 
    let mut Mt = Matrix::zeros(2,2);
    Mt[(0,0)] =    a.0*v.powi(2) + a.1*w.powi(2);     
    Mt[(1,1)] =    a.2*v.powi(2) + a.3*w.powi(2);

    let mut Vt = Matrix::zeros(3,2);
    Vt[(0,0)] = cos(Theta)*delta_T;
    Vt[(1,0)] = sin(Theta)*delta_T;
    Vt[(2,1)] = delta_T;
    mat_mul(&Vt,&mat_mul(&Mt,&Vt.transpose()))
}











// mean_prev
// cov_prev
// cmd_vel
// obs
// detector
#[allow(non_snake_case,unused)]
pub fn ekf_slam_step(
            params:&mut EkfSLAMParams,
            R:&mut Matrix,
            Q:&mut Matrix,
            mean_prev:&mut Matrix,
            cov_prev:&mut Matrix,
            cmd_vel:&mut Matrix,
            obs_vec:&mut Vec<Matrix>,
            did_see:&mut Vec<bool>
                    ){    
    // From here are initialised values 
    let num_obsts = params.num_obsts;
    let num_params_movt = params.num_params_movt;
    let num_params_obst = params.num_params_obst  ;
    let num_elements =  params.num_elements;
    let delta_T  = params.delta_T;
    
    // refactor so as to not rewrite all the time 
    let mut Fx = Matrix::zeros(num_params_movt,num_elements);
    let mut I = Matrix::identity(num_elements,num_elements);
    Fx[(0,0)] = 1.;
    Fx[(1,1)] = 1.;
    Fx[(2,2)] = 1.;
    // ^refactor these 
        
    //velocity update  -O
    let v = cmd_vel[mX];
    let mut w = cmd_vel[mY];
    let T_prev = mean_prev[mT];
    

    let mut vel_update =  vel_motion_update(v,w,delta_T,T_prev);
    let mut mean_t = mat_add(&mean_prev, &mat_mul(&Fx.transpose(),&vel_update)); 
    
 
    // maybe do R update function here 
    //jacobian update  -O
    let mtest = 0.001;
    let Rt =  get_rt((mtest,mtest,mtest,mtest),v,w,T_prev,delta_T);
    print!("Rt is");
    print_matrix(&Rt);

    let mut jacobian_update =  vel_jacobian_update(v,w,delta_T,T_prev);
    let Gt = mat_add(&I , &mat_mul(&Fx.transpose(),&mat_mul(&jacobian_update,&Fx)));
    let mut cov_t = mat_add( &mat_mul(&Gt,&mat_mul(&cov_prev,&Gt.transpose())), &mat_mul(&Fx.transpose(),&mat_mul(&Rt, &Fx)));
    
 
    obs_vec.clear();
    // observation integration 
    for m in obs_vec.iter(){
        //println!("cov_t : ");
        //print_matrix(&cov_t);
        //actual observations
        let mut z_obs = Matrix::zeros(num_params_obst,1);
        z_obs[mX] = m[mX];
        z_obs[mY] = con_bear(m[mY]);
        let index = m[mT] as usize;     // could be an error, you're turning float into int 
        
        let mat_index = num_params_movt + num_params_obst*index;
        let T_t = mean_t[mT];
       

        if did_see[index]==false{
            let phi = con_bear(z_obs[mY] +T_t);
            mean_t[(mat_index,0)] = mean_t[mX] + z_obs[mX]*cos(phi); 
            mean_t[(mat_index+1,0)] = mean_t[mY] + z_obs[mX]*sin(phi); 
            did_see[index] = true;
        }
        

        let delta_x = mean_t[(mat_index,0)]  - mean_t[mX];
        let delta_y = mean_t[(mat_index+1,0)]  - mean_t[mY];
        
        let q = delta_x.powi(2) + delta_y.powi(2);
        let q_r = q.sqrt();
        let mut z_t = Matrix::zeros(num_params_obst,1);
        let mut Fxj = Matrix::zeros(num_params_movt+num_params_obst,num_elements);
        Fxj[(0,0)] = 1.;
        Fxj[(1,1)] = 1.;
        Fxj[(2,2)] = 1.;
        
        Fxj[(3,mat_index)] = 1.;
        Fxj[(4,mat_index+1)] = 1.;

        
        z_t[(0,0)] = q_r;
        z_t[(1,0)] = con_bear(Float::atan2(delta_y,delta_x)  - T_t);

        
        let mut Ht = Matrix::zeros(num_params_obst,num_params_movt+num_params_obst);
        Ht[(0,0)] = -delta_x*q_r;
        Ht[(0,1)] = -delta_y*q_r;
        Ht[(0,2)] = 0.; 
        Ht[(1,0)] = delta_y;
        Ht[(1,1)] = -delta_x;
        Ht[(1,2)] = -q;
        
        
        Ht[(0,3)] = delta_x*q_r;
        Ht[(0,4)] = delta_y*q_r;
        Ht[(1,3)] = -delta_y;
        Ht[(1,4)] = delta_x;
        
        Ht = (1.0/q)*Ht ;
        Ht = mat_mul(&Ht,&Fxj);
        // forgot to multiply by 1/q
        
        let K = mat_mul( &mat_mul(&cov_t,&Ht.transpose()), &mat_add( &mat_mul(&mat_mul(&Ht,&cov_t),&Ht.transpose()) , &Q   ).try_inverse().expect("Unable to invert matrix"));
       
        
        // TODO (jfarhan):Delete
        let T_Debug = &mat_add(    &mat_mul( &mat_mul(&Ht,&cov_t),&Ht.transpose())  , &Q).try_inverse().expect("Unable to invert matrix"); 
        
        // TODO(jfarhan):Delete
        let K_Debug = mat_mul( &mat_mul(&cov_t,&Ht.transpose()) , &T_Debug );
        
        //TODO (jfarhan):Delete
        let diff_factor = &mat_mul(&K,&mat_sub(&z_obs,&z_t));
        
        //println!("DIFF FACTOR");
        //print_matrix(&diff_factor);
        
        // TODO (jfarhan):Delete
        //println!("K matrix");
        //print_matrix(&K);
        print!("H MATRIX");
        print_matrix(&Ht);



        mean_t = mat_add(&mean_t,&mat_mul(&K,&mat_sub(&z_obs,&z_t)));
        cov_t = mat_mul(&mat_sub(&I,&mat_mul(&K,&Ht)),&cov_t)
        // do the rest 
    } 
    *mean_prev = mean_t;
    *cov_prev = cov_t;
}










// NOTE - tests
// start
// here 
#[cfg(test)]
pub mod tests{
    use std::fs::File;
    use std::io::{BufReader,BufRead};
    use crate::simple_ekf_slam::{Matrix,EkfSLAMParams,mX,mY,mT,odom_get_v_w,ekf_slam_step,print_matrix,con_bear};




    pub fn file_read_odom_accurate(path_odometry:&std::path::Path, path_abs:&std::path::Path)->std::io::Result<Vec<( (f32,f32),(f32,f32) )>>{
        let mut odomfile = File::open(path_odometry)?;
        let mut odom_reader  = BufReader::new(&mut odomfile);
        let mut odom_str = String::new();

        let mut absfile = File::open(path_abs)?;
        let mut abs_reader  = BufReader::new(&mut absfile);
        let mut abs_str = String::new();
        let mut output_vec = Vec::<((f32,f32),(f32,f32))>::new();

        loop{
            let mut datas = ((0.,0.),(0.,0.));
            if let Ok(p) = odom_reader.read_line(&mut odom_str){
                if p==0{
                    break;
                }
                odom_str = odom_str.replace("\r\n","");
                //println!("ODOM STR {}",odom_str);
                let mut odom_iter = odom_str.split("|");
                let odom_l = odom_iter.next().unwrap().parse::<f32>().unwrap_or_else(|_|{0.0});
                let odom_r = odom_iter.next().unwrap().parse::<f32>().unwrap_or_else(|_|{0.0});
                datas.0.0=odom_l;
                datas.0.1=odom_r;
                //println!("ODOM {},{}",odom_l,odom_r);
                odom_str.clear();
            }else{
                println!("EOF");
                break;
            }

            if let Ok(p) = abs_reader.read_line(&mut abs_str){
                if p==0{
                    break;
                }
                abs_str = abs_str.replace("\r\n","");
                let mut abs_iter = abs_str.split("|");
                let x = abs_iter.next().unwrap().parse::<f32>();
                let y = abs_iter.next().unwrap().parse::<f32>();
                datas.1.0=x.unwrap_or_else(|_|{0.0});
                datas.1.1=y.unwrap_or_else(|_|{0.0});
                //println!("ABS {},{}",datas.1.0,datas.1.1);
                abs_str.clear();
            }else{
                break;
            }
            output_vec.push(datas);
        }
        
        Ok(output_vec)
    }
    
   



    use crate::simple_ekf_slam::{vel_motion_update,vel_motion_update_matlab};
    use std::io::Write;
    #[test]
    fn test_velocity_model(){
        let mut f = std::fs::File::create("velocity_model_test_new.txt").unwrap();
        let mut f2 = std::fs::File::create("velocity_model_test_old.txt").unwrap();
        
        let mut writer = std::io::BufWriter::new(&mut f);
        let mut writer2 = std::io::BufWriter::new(&mut f2);
        
        let odom_data = file_read_odom_accurate( std::path::Path::new("raw_data/ws_pos8008.txt"),std::path::Path::new("raw_data/abs_pos8008.txt")).unwrap();         
        let mut init = Matrix::zeros(3,1);
        let mut odom_new = (0.,0.);
        let mut odom_prev = (0.,0.);
        let base_length = 0.1054;
        let wheel_radius = 0.021;
        
        let mut coords_nm = (0.,0.,1.57);
        let mut coords = (0.,0.,1.57);
        odom_data.iter().map(|x|{
            let odom_new = x.0;
            let v_w = odom_get_v_w(odom_prev,odom_new ,base_length,wheel_radius); 
            let c  = vel_motion_update_matlab(v_w.0,v_w.1,1.0,coords.2);
            coords.0 = coords.0 + c[mX];
            coords.1 = coords.1 + c[mY];
            coords.2 = coords.2 + c[mT];
            odom_prev = odom_new;


            let cz  = vel_motion_update(v_w.0,v_w.1,1.0,coords.2);
            coords_nm.0 = coords_nm.0 + cz[mX];
            coords_nm.1 = coords_nm.1 + cz[mY];
            coords_nm.2 = coords_nm.2 + cz[mT];
            odom_prev = odom_new;
            writeln!(&mut writer,"{}|{}|{}",coords.0,coords.1,coords.2);
            writeln!(&mut writer2,"{}|{}|{}",coords_nm.0,coords_nm.1,coords_nm.2);
        }).count();
    }
   


    static base_length_:f32 = 0.1054;
    static wheel_radius_:f32= 0.021;
    fn test_velocity_model_step(coords:&mut (f32,f32,f32),odom_prev:&mut (f32,f32),odom_new:(f32,f32)){
            let v_w = odom_get_v_w(*odom_prev,odom_new ,base_length_,wheel_radius_); 
            let c  = vel_motion_update_matlab(v_w.0,v_w.1,1.0,coords.2);
            coords.0 = coords.0 + c[mX];
            coords.1 = coords.1 + c[mY];
            coords.2 = coords.2 + c[mT];
            *odom_prev = odom_new;
    }


    extern crate probability;
    use probability::prelude::*;
    fn simulate_test_objects(x:f32,y:f32,theta:f32,index:usize) -> Vec<Matrix>{
        //adding gaussian noise
        let mut source = source::default();
        let distribution = Gaussian::new(0.,0.04);
        let mut sampler = Independent(&distribution,&mut source);


        let objs = vec![(2.5,2.5),(-2.5,2.5),(-2.5,-2.5),(2.5,-2.5)];
        objs.iter().enumerate().filter(|m|{ ((m.1.0-x).powi(2) +  (m.1.1-y).powi(2)) < 4.0}).map(|m|{
            let mut z = Matrix::zeros(3,1);
            let x_o = m.1.0;
            let y_o = m.1.1;

            let r = ((x-x_o).powi(2) + (y-y_o).powi(2)).sqrt().abs(); 
            

            let r_sampled = r + sampler.next().unwrap() as f32;
            
            let phi =  f32::atan2( y_o-y , x_o-x )  - theta ; 
            let phi_sampled = phi + sampler.next().unwrap() as f32;
            
            z[mX] = r_sampled;              
            z[mY] = con_bear(phi_sampled);
            z[mT] = m.0 as f32;
            //println!("[{}]INPUT: r:{},phi:{}",index,r,con_bear(phi));
            //println!("[{}]SAMPLED: r:{},phi:{}",index,r_sampled,con_bear(phi_sampled ));
            z
        }).collect::<Vec<Matrix>>() 
    }


        

    #[test]
    fn simple_ekf_slam_test(){    
        let num_obsts = 5;
        let num_params_movt = 3;
        let num_params_obst = 2  ;
        let delta_T  = 1.0;
        let mut params = EkfSLAMParams::new(num_params_movt,num_params_obst,num_obsts,delta_T);
        
        let num_elements = params.num_elements;
        

        // Motion model error matrix 
        let mut R = Matrix::identity(num_params_movt,num_params_movt); 
        R[(0,0)] = 0.001;
        R[(1,1)] = 0.001;
        R[(2,2)] = 0.001;



        // Sensor model error matrix
        let mut Q = Matrix::identity(num_params_obst,num_params_obst);
        Q[(0,0)] = 12.0;
        Q[(1,1)] = 12.0;


        let mut mean_prev = Matrix::zeros(num_elements,1);
        mean_prev[mT] = 1.57;
        print_matrix(&mean_prev);


        let mut cov_prev =  Matrix::zeros(num_elements,num_elements);
        (num_params_movt..num_elements).map(|m|{
            cov_prev[(m,m)] = 100.0;
        }).count();
        print_matrix(&cov_prev);
        

        let mut cmd_vel = Matrix::zeros(2,1);
        let mut obs_vec = (0..3).map(|x|{let x = x as f32;let mut z = Matrix::zeros(3,1);z[mX] = x*2.*5.;z[mY]=x*2.;z[mT]=x;
            z}).collect::<Vec<Matrix>>();
        let mut did_see = vec![false;4];
        obs_vec.clear();



        let base_length = 0.1054;
        let wheel_radius = 0.021;
        let mut odom_prev = (0.,0.);

        let mut odom_prev_ = (0.,0.);
        let mut coords_ = (0.,0.,1.57);
        
        let mut abs_state = (0.,0.);



        let odom_data = file_read_odom_accurate( std::path::Path::new("raw_data/ws_pos8008.txt"),std::path::Path::new("raw_data/abs_pos8008.txt")).unwrap();         
        
        odom_data.iter().enumerate().map(|m|{
            let index = m.0;
            let odom_new = m.1.0;
            let v_w = odom_get_v_w(odom_prev,odom_new ,base_length,wheel_radius); 
            cmd_vel[mX] = v_w.0;
            cmd_vel[mY] = v_w.1;
            test_velocity_model_step(&mut coords_ ,&mut odom_prev_,odom_new);
                
            let x_abs =  m.1.1.0;
            let y_abs = -m.1.1.1;
            let angle_abs = con_bear(f32::atan2(y_abs - abs_state.1,x_abs-abs_state.0));
            obs_vec = simulate_test_objects(x_abs,y_abs,angle_abs,index ) ;
            abs_state =(x_abs,y_abs);

            ekf_slam_step(
                &mut params,
                &mut R,
                &mut Q,
                &mut mean_prev,
                &mut cov_prev,
                &mut cmd_vel,
                &mut obs_vec,
                &mut did_see
                );

            // create observation matrix data 
            println!("\n[{}]ekf:({},{},{})",index,mean_prev[mX],mean_prev[mY],mean_prev[mT]);
            println!("[{}]velocity_model:{:?}",index,coords_);
            println!("[{}]abs:({} , {})",index,x_abs,y_abs);
            
            odom_prev = odom_new;
        }).count();
            
        print_matrix(&mean_prev);
    }
   



    use crate::simple_ekf_slam::mat_sub;
    #[test]
    fn random_test(){
        let mut mat_A = Matrix::zeros(2,2);
        let mut mat_B = Matrix::zeros(2,2);
        
        mat_A[(0,0)] = 3.;
        mat_A[(1,1)] = 3.;

        mat_B[(0,0)] = 2.;
        mat_B[(1,1)] = 2.;
        mat_B = mat_B*2.4;
        
        mat_A = 1.2*mat_A;
        print_matrix(&mat_B);
        print_matrix(&mat_A);

        let mat_C = mat_sub(&mat_A,&mat_B); 
        print_matrix(&mat_C);
    }
}

/*
 * Some sources of errors:
 * 1. Jacobians might not be updating properly ; use andrewjkramers motion model to test
 * 2. I cannot find any other errors at this point in time  
 * */
