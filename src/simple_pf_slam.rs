use std::collections::HashSet;
type Float = f32;
static _2PI :Float = std::f32::consts::PI * 2.0;
pub static mX:(usize,usize) = (0,0);
pub static mY:(usize,usize) = (1,0);
pub static mT:(usize,usize) = (2,0);
static mDS:(usize,usize) = (3,0);


static jX:(usize,usize) = (0,2);
static jY:(usize,usize) = (1,2);
static jT:(usize,usize) = (2,2);


static mR:(usize,usize) = (0,0);
static mPhi:(usize,usize) = (1,0);



use crate::simple_ekf_slam::{sin,cos};
use na::DMatrix;
pub type Matrix = DMatrix<f32>;
static X:usize = 0;





use probability::prelude::*;
pub fn sample_velocity(delta_T:Float,v:Float,w:Float,state:&(Float,Float,Float), alphas:&(Float,Float,Float,Float,Float,Float))->(Float,Float,Float){
    let mut source = source::default();
    let v_var = alphas.0*v*v + alphas.1*w*w;
    let w_var = alphas.2*v*v + alphas.3*w*w;
    let g_var = alphas.4*v*v + alphas.5*v*v;

    let v_distr =  Gaussian::new(0.,v_var as f64 );
    let w_distr =  Gaussian::new(0.,w_var as f64 );
    let g_distr =  Gaussian::new(0.,g_var as f64 );
    
    let sampled_v =  Independent(&v_distr,&mut source).next().unwrap() as f32;
    let sampled_w =  Independent(&w_distr,&mut source).next().unwrap() as f32;
    let sampled_g =  Independent(&g_distr,&mut source).next().unwrap() as f32;
    let v_ = v + sampled_v;
    let w_ = w + sampled_w;
    let g_ =sampled_g;
    let x = state.0;
    let y = state.1;
    let T = state.2;

    if w==0.0{
        let x_ = x + v_*cos(T);
        let y_ = y + v_*sin(T);
        let T_ = g_*delta_T;
        (x_,y_,T_)
    }else{
        let x_ = x - (v_/w_)*sin(T) + (v_/w_)*sin(T + w_*delta_T);
        let y_ = y + (v_/w_)*cos(T) - (v_/w_)*cos(T + w_*delta_T);
        let T_ = T + w_*delta_T + g_*delta_T;
        (x_,y_,T_)
    }

}






pub fn particle_filter_step(){
    let particles:Vec<Box<Vec<Matrix>>> = Vec::new();
    //  for each particle A
    //      sample a new pose from A
    //      for eeach observed measurement:
    //          measurment update  B, update mean and covariance:
    //              z_t  + H_t (m_c - \mu_c)
    //              find K 
    //              \mu_c = \mu_c + K(zt - z_bar) 
    //              \Cov_c = (1 - KH)\cov_c
    //      
    //      calculate impotance weight from B 
    //      resample 
}



// M particles 
#[allow(unused)]
pub fn fastSLAM_step(
    Z:&mut Vec<(Matrix,usize)>,
    did_see:&mut Vec<bool>,
    U:&mut Matrix,
    prev:Vec<Vec<Matrix>>){
    
    let mut sampled :Vec<Vec<Matrix>> = Vec::new();
    let mut newly_seen:HashSet<usize> =HashSet::new();
    
    let alpha_f = 0.02; 
    let alphas = (alpha_f,alpha_f,alpha_f, alpha_f,alpha_f,alpha_f);



    prev.into_iter().map(|mut m|{
        let state = (m[0][mX],m[0][mX],m[0][mT]);
        let v = U[mX];
        let w = U[mY];
        let X_t = sample_velocity(1.0 ,v,w,&state,&alphas);

        Z.iter().map(|(obs,index)|{
            // if feature never seen before 
            if !did_see[*index]{

                // TODO (jfarhan) : push a mean and covariance 
                let x_ = X_t.0;
                let y_ = X_t.1;
                let t_ = X_t.2;
                
                let r = obs[mX];
                let phi = obs[mY] + t_;

                let mut mean_Zi = Matrix::zeros(2,1);
                mean_Zi[mX] = x_ + r*cos(phi);
                mean_Zi[mY] = y_ + r*sin(phi);
                m.push(mean_Zi);
    
                let mut H = Matrix::zeros(2,2);
                // TODO (jfarhan): create H matrix
                newly_seen.insert(*index);
            }
            // TODO (jfarhan) :if already seen conditions
        }).count();
    }).count();
    


    newly_seen.into_iter().map(|x|{
        did_see[x] = true;
    }).count();

}



















#[cfg(test)]
mod tests{

    use crate::simple_pf_slam::{Matrix,X,sample_velocity};
    #[test]
    pub fn pf_test(){
        let mut particle : Vec<Matrix> = Vec::new();
        let num_obs = 4;
        particle.push( Matrix::zeros(3,1)) ;
        (0..num_obs).for_each(|_|{
            particle.push(Matrix::zeros(2,1));
            particle.push(Matrix::zeros(2,2));
        });

    }


    use std::io::{Write};
    #[test]
    fn sample_velocity_test(){
        let state = (0.,0.,0.0);
        let delta_T = 1.0;
        let v = 0.1;
        let w = 0.1;
        let alphas = (0.6,0.6,0.06, 0.06,0.06,0.06);
        let mut f = std::fs::File::create("sampling_test.txt").unwrap();
        let mut writer = std::io::BufWriter::new(&mut f);        
        (0..500).map(|x|{
            let m = sample_velocity(delta_T,v,w,&state, &alphas);
            writeln!(&mut writer,"{}|{}|{}",m.0,m.1,0.0);
            println!("x = {} , y = {}",m.0,m.1);
        }).count();

    }
}
