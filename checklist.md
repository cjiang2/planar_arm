# Planar Robot
- [] Base: 3-axis
    - [OK] Homogeneous transformation for Forward Kinematics.
    - [OK] Stick Visualization.
    - [] Numerical Inverse Kinematics.
        - [] Analytical Jacobian.
        - [OK] Jacobian w/ Central Difference.
        - [OK] Newton's Method.
        - [OK] Broyden's Method.
    - [] Demo-able.
        - [OK] Keyboard control.
        - [] Varying trajectories. 
            - [] Suggestion from Martin 1st.

- [] Variant 1: 3 more rotational DoF.
    - [] Ask Martin.

- [] Visual Servoing
    - [] Camera Projection.
        - [] Reading: Figure out which camera model to use.
        - [] Specify one or two cameras.
            - [] On arm camera.
            - [] Static camera.

    - [] Update Inverse Kinematics with Image Errors.

- [] Analytical: Error in Kinematics.
    - [] How does an error \Delta q in robot joint space propagate to a mm error in 3D Euclidean coordinates?
        - Potential connections to planar_contrastive.

    - [] Experimental: Try different \Delta q (and \Delta l - link lengths) and see how end pose varies.