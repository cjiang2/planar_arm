# Planar Robot
- [] Base: 3-axis
    - [OK] Homogeneous transformation for Forward Kinematics.
    - [OK] Stick Visualization.
    - [OK] Numerical Inverse Kinematics.
        - [OK] Analytical Jacobian.
        - [OK] Jacobian w/ Central Difference.
        - [OK] Newton's Method.
        - [OK] Broyden's Method.
    - [] Demo-able.
        - [OK] Keyboard control.
        - [] Varying trajectories. 
            - [] Suggestion from Martin 1st.
    
    - [] Debug.
        - [] Fix the orientation.

- [] Variant 1: 3 more rotational DoF.
    - [] Ask Martin.

- [ok] Visual Servoing
    - [ok] Camera Projection.
        - [] Reading: Figure out which camera model to use.
            - camera up vector.
        - [] Specify one or two cameras.
            - [] On arm camera.
            - [] Static camera.

    - [] Update Inverse Kinematics with Image Errors.

- [] Analytical: Error in Kinematics.
    - [] How does an error \Delta q in robot joint space propagate to a mm error in 3D Euclidean coordinates?
        - Potential connections to planar_contrastive.

    - [] Experimental: Try different \Delta q (and \Delta l - link lengths) and see how end pose varies.


        delta_q indicates changes in poses,
        delta_y indicates changes from visual space(visual servoing) 

        see the conditional

        delta y = J delta q
        delta q = inv(J) delta y
        cond analysis for the control equation on book

        Jk+1 = Jk + (delta y - Jx deltaq) delta q.T / (some terms here)

        measure pose error, image error, conditional number of the jacobian
        for broyden, we can introduce noise in delta q, or delta y, 
        choose to initialize jacobian in a wrong direction.

        alpha doesn't change explicit gain for the control equation in general.
        (experiment this)

- [] Think about
Grasp sphere: formulate a constraint x+ C x = 0