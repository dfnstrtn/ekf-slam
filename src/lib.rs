//pub mod differential_drive;
extern crate nalgebra as na;
pub mod ekf_slam;
#[cfg(test)]
pub mod tests;




pub mod array_test{

    use ndarray::Array2;
    use ndarray::arr2;
    
    pub fn test_init(){
        let mut data  = Array2::<f64>::zeros((3,3));
        data[(1,1)] = 0.5;
        data[(0,0)] = 155.1;
        data[(0,1)] = 16.1;
        data[(0,2)] = 15.1;
        data[(1,0)] = 11.1;
        data[(1,2)] = 21.1;


        println!("{:?}",data.shape());
        for row in data.genrows(){
            println!("{:?}",row);
        }

        println!("{:?}",data[[1,1]]);
        println!("{}",data.sum());

    
        let mut testarr = arr2(&[[1,2],[3,4]]);
        let mut testarr2 = arr2(&[[11,21],[31,41]]);
        let mut c  = testarr.dot(&testarr2);

        println!("{:?}",c);


    }
}




pub mod nalgebra_test{
    use na::{U2,U3,Dynamic,ArrayStorage,VecStorage,Matrix};
    pub fn test_matrix(){
        let mut test_matrix = na::DMatrix::<f32>::identity(3,3);
        test_matrix[(0,0)] = 0.5;
        test_matrix[(0,1)]=0.6;
        test_matrix[(0,2)]=0.7;
        test_matrix[(1,1)]=0.9;
        test_matrix[(2,1)]=0.89;

       test_matrix.row_iter().for_each(|m|{
           m.iter().for_each(|n|{
               print!("{} ",n);
           });
           println!("");
       });
        
    
       println!("transpose");
       let m_new = test_matrix.transpose();


       m_new.row_iter().for_each(|m|{
           m.iter().for_each(|n|{
               print!("{} ",n);
           });
           println!("");
       });
       
      
        println!("inverse");
       let nm_new = test_matrix.try_inverse().unwrap();


       nm_new.row_iter().for_each(|m|{
           m.iter().for_each(|n|{
               print!("{} ",n);
           });
           println!("");
       });

    }

}






pub mod slam{
    

}



