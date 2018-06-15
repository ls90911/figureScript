
Information for generating the article figures and extracting data from the datasets.
Scripts are sorted between Vision, State estimation and Control
After running a script, the article plots are saved in the autosave folder


Instructions for generating article plots:

-Vision

    Goto vision folder and add the func_common folder and sub folders to the matlab path

    Run the following scripts:
    
    //Comparing Least squares with P3p position estimation in simulation 
    Vision_test_POS_compared.m

    //Comparing Least squares with P3p position estimation in simulation
    Vision_test_HEADING_compared.m

    //Histogram position simulation
    histogram_pos_test.m

    //test gate based heading estimation on real data
    corner_processing_vision_heading.m

-State estimation

    Run the following scripts:

    //Loop flight, post processing on-board data
    EKF_drag_2_1_Final.m


    //Basement flight using on-board position estimation
    EKF_drag_2_1_Final_loop_track.m

-Control
    
    Goto vision folder and add the func_common folder and sub folders to the matlab path

    Run the following scripts:

    //Analyse large number of open loop arcs form on-board data
    Arc_test_2.m



