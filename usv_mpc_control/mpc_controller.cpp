#include "mpc_controller.h"

mpc_output mpc_controller_3dof(struct vehicle_local_position_s *local_pos,
                               struct vehicle_attitude_s *vehicle_att,
                               float U_L, float U_R,
                               matrix::Matrix<float, N_STATE, 1> prev_state_vector,
                               float desired_surge, float desired_yaw)
{


    float yaw_angle = matrix::Eulerf(matrix::Quatf(vehicle_att->q)).psi();


    mpc_output Return;

    static auto B_constant = Create_B_Matrix();
    static auto C = Create_C_Matrix();
    static auto Q = Create_Q_Matrix();
    static auto R = Create_R_Matrix();
    static auto Eye_Disc = Create_Eye_For_Disc();
    static auto A_Constraints = Create_A_Constraints();
    static auto b_Constraints = Create_b_Constraints();
    //A_Constraints.print();
    //matrix::Matrix<float, 2*N_C*N_I, 1> b_Constraints;
    b_Constraints.setZero();

    int remainder_row = 0;
    // Construct A_Constraints Matrix
    for (int row = 0; row < N_C*N_I; row++)
    {
        remainder_row = row % 2;
        if((int)remainder_row == 0)
        {
            b_Constraints._data[row][0] = U_MAX - U_L;
            b_Constraints._data[N_C*N_I+row][0] = -U_MIN + U_L;
        }
        if((int)remainder_row == 1)
        {
            b_Constraints._data[row][0] = U_MAX - U_R;
            b_Constraints._data[N_C*N_I+row][0] = -U_MIN + U_R;
        }
    }
    //b_Constraints.print();

    float vx = 0.0f;
    float vy = 0.0f;
    float r = vehicle_att->yawspeed;
    vx = (local_pos->vx)*(float)cos(yaw_angle) + (local_pos->vy)*(float)sin(yaw_angle);
    vy = -(local_pos->vx)*(float)sin(yaw_angle) + (local_pos->vy)*(float)cos(yaw_angle);


    matrix::Matrix<float, N_STATE, 1> current_state_vector;
    current_state_vector.setZero();

    matrix::SquareMatrix<float, N_STATE> A;
    /* initially set all the element zero */
    A.setZero();

//    /* Indicies of A Matrix */
    // First Row
    A._data[0][2] = -vy*(float)cos(yaw_angle) - vx*(float)sin(yaw_angle);
    A._data[0][3] = (float)cos(yaw_angle);
    A._data[0][4] = -(float)sin(yaw_angle);
    A._data[0][6] = vx*(float)cos(yaw_angle) - vy*(float)sin(yaw_angle);

    //Second Row
    A._data[1][2] = vx*(float)cos(yaw_angle) - vy*(float)sin(yaw_angle);
    A._data[1][3] = (float)sin(yaw_angle);
    A._data[1][4] = (float)cos(yaw_angle);
    A._data[1][6] = vy*(float)cos(yaw_angle) + vx*(float)sin(yaw_angle);


    // Third Row
    A._data[2][5] = 1.0f;
    A._data[2][6] = r;

    // Fourth Row
    A._data[3][3] = -0.07112f*vx - 0.2135f*(float)fabs(vx)
                    -0.2135f*vx*signum(vx) - 0.4073f;
    A._data[3][4] = 0.7828f*r;
    A._data[3][5] = 0.7828f*vy - 0.01044f*r;
    A._data[3][6] = 0.07872f*U_L + 0.07872f*U_R
                    -0.07872f*vx*(2.7123f*(float)fabs(vx) + 5.1740f)
                    -0.07872f*r*(0.06634f*r - 9.9441f*vy) - 0.03555f*vx*vx;

    // Fifth Row
    A._data[4][3] = 0.0049f*vy - 0.7721f*r;
    A._data[4][4] = 0.0049f*vx - 0.1499f*vy - 0.3215f*(float)fabs(vy)
                    -0.3215f*vy*signum(vy) - 0.1789f;
    A._data[4][5] = 0.0004786f*(float)fabs(r) - 0.7721f*vx
                    +0.0004786f*r*signum(r) + 0.0003525f;
    A._data[4][6] = 0.001894f*U_R - 0.001894f*U_L
                    -0.000008299f*r + 0.0000007579f*vy
                    +0.007578f*r*(0.06316f*(float)fabs(r) + 0.04762f)
                    -0.08299f*vy*(3.8741f*abs(vy) + 2.1562f) - 0.7716f*r*vx
                    -0.07046f*vx*vy - 0.007579f*vx*(0.06634f*r - 9.9441f*vy)
                    -0.07497f*vy*vy;

   // Sixth Row
    A._data[5][3] =  0.1618f*r - 0.8905f*vy;
    A._data[5][4] = 0.01369f*vy - 0.8905f*vx
                    +0.02936f*(float)fabs(vy) + 0.02936f*vy*signum(vy) + 0.0162f;
    A._data[5][5] = 0.1618f*vx - 0.08698f*(float)fabs(r)
                    -0.08698f*r*signum(r) - 0.06558f;
    A._data[5][6] = 0.3443f*U_L - 0.3443f*U_R - 1.541f*vy
                    +0.0000007579f*r - 0.0001377f*vy
                    -1.3377f*r*(0.06316f*(float)fabs(r) + 0.04762f)
                    +0.007578f*vy*(3.8741f*(float)fabs(vy) + 2.1562f)
                    +0.07046f*r*vx + 12.8052f*vx*vy +
                    +1.3772f*vx*(0.06634f*r - 9.9441f*vy)
                    +0.006846f*vy*vy;

    // Discretization from continuous to dimpc_returnscrete time system
    A = sampling_time * A + Eye_Disc;
    auto B = sampling_time * B_constant;

    matrix::Matrix<float,N_OUTPUT+N_STATE, N_OUTPUT+N_STATE> A_e;
    matrix::Matrix<float,N_OUTPUT+N_STATE, N_I> B_e;
    matrix::Matrix<float,N_OUTPUT, N_OUTPUT+N_STATE> C_e;
    matrix::Matrix<float,N_OUTPUT, N_STATE> dummy_C_A;
    matrix::Matrix<float,N_OUTPUT, N_I> dummy_C_B;
    matrix::Matrix<float,N_OUTPUT, N_OUTPUT+N_STATE> dummy_Ce_Ae;
    matrix::Matrix<float,N_OUTPUT, N_I> dummy_Ce_Ae_B_e;
    matrix::Matrix<float,N_OUTPUT*N_P, N_OUTPUT+N_STATE> F;
    matrix::Matrix<float,N_OUTPUT*N_P, N_I*N_C> Phi;

    A_e.setIdentity();
    B_e.setZero();
    C_e.setZero();
    F.setZero();
    Phi.setZero();

    // A_e calculation
    for (int row = 0; row < N_STATE; row++) {
        for (int column = 0; column < N_STATE; column++) {
            A_e._data[row][column] = A._data[row][column];
        }
    }

    dummy_C_A = C*A;
    for (int row = 0; row < N_OUTPUT; row++) {
        for (int column = 0; column < N_STATE; column++) {
            A_e._data[N_STATE+row][column] = dummy_C_A._data[row][column];
        }
    }
    //----------------------------------------------------------------------
    //PX4_INFO("A_e MATRIX");
    //A_e.print();

    // B_e calculation
    for (int row = 0; row < N_STATE; row++) {
        for (int column = 0; column < N_I; column++) {
            B_e._data[row][column] = B._data[row][column];
        }
    }

    dummy_C_B = C*B;
    for (int row = 0; row < N_OUTPUT; row++) {
        for (int column = 0; column < N_I; column++) {
            B_e._data[N_STATE+row][column] = dummy_C_B._data[row][column];
        }
    }
    //----------------------------------------------------------------------

    // C_e calculation
    for (int row = 0; row < N_OUTPUT; row++){
            C_e._data[row][N_STATE+row] = 1;
    }
    //----------------------------------------------------------------------
    //PX4_INFO("C_e MATRIX");
    //C_e.print();

    dummy_Ce_Ae = C_e * A_e;
    dummy_Ce_Ae_B_e = C_e * B_e;

    // F Matrix Calculation
    for (int row = 0; row < N_OUTPUT; row++) {
        for (int column = 0; column < N_OUTPUT+N_STATE; column++) {
            F._data[row][column] = dummy_Ce_Ae._data[row][column];
        }
    }
    for (int index = 1; index <= N_P-1; index++) {
        dummy_Ce_Ae = dummy_Ce_Ae*A_e;

        for (int row = 0; row < N_OUTPUT; row++) {
            for (int column = 0; column < N_OUTPUT+N_STATE; column++) {
                F._data[N_OUTPUT*index+row][column] = dummy_Ce_Ae._data[row][column];
            }
        }
    }
    //----------------------------------------------------------------------

    // Phi Matrix Calculation
    for (int row = 0; row < N_OUTPUT; row++) {
        for (int column = 0; column < N_I; column++) {
            Phi._data[row][column] = dummy_Ce_Ae_B_e._data[row][column];
        }
    }

    for (int index = 1; index <= N_P-1; index++) {

        //F.slice<N_OUTPUT,N_OUTPUT+N_STATE>((index-1)*2,0).print();
        dummy_Ce_Ae_B_e = F.slice<N_OUTPUT,N_OUTPUT+N_STATE>((index-1)*N_OUTPUT,0)*B_e;

        for (int row = 0; row < N_OUTPUT; row++) {
            for (int column = 0; column < N_I; column++) {
                Phi._data[N_OUTPUT*index+row][column] = dummy_Ce_Ae_B_e._data[row][column];
            }
        }
    }

    for (int column_index = 1; column_index <= N_C-1; column_index++){
        int k = 0;
        for (int row_index = column_index; row_index <= N_P-1; row_index++){

            //Phi.slice<N_OUTPUT,N_I>(k*2,0).print();
            //dummy_Ce_Ae_B_e = Phi.slice<N_OUTPUT,N_I>(k*2,0);

            for (int row = 0; row < N_OUTPUT; row++) {
                for (int column = 0; column < N_I; column++) {
                    //Phi._data[row_index*N_OUTPUT+row][column_index*N_I+column] =
                    //                            dummy_Ce_Ae_B_e._data[row][column];
                    Phi._data[row_index*N_OUTPUT+row][column_index*N_I+column] =
                                                          Phi._data[k*N_OUTPUT+row][column];
                }
            }
        k = k + 1;
        }
    }
    //PX4_INFO("Phi MATRIX");
    //Phi.print();
    matrix::SquareMatrix<float, N_C*N_I> K_Matrix;
    K_Matrix.setZero();
    matrix::SquareMatrix<float, N_C*N_I> inv_K_Matrix;
    K_Matrix = (Phi.transpose()*Q*Phi + R) * 2;
    //PX4_INFO("K MATRIX");
    //K_Matrix.print();
    inv_K_Matrix = matrix::inv(K_Matrix);
    //inv_K_Matrix.print();

    current_state_vector._data[0][0] = local_pos->x;
    current_state_vector._data[1][0] = local_pos->y;
    current_state_vector._data[2][0] = yaw_angle;
    current_state_vector._data[3][0] = vx;
    current_state_vector._data[4][0] = vy;
    current_state_vector._data[5][0] = r;
    current_state_vector._data[6][0] = 1.0f;

    matrix::Matrix<float, N_STATE+N_OUTPUT, 1> augmented_x;
    for (int row = 0; row < N_STATE; row++) {
        augmented_x._data[row][0] = current_state_vector._data[row][0] -
                                    prev_state_vector._data[row][0];
    }
    augmented_x._data[N_STATE][0] = current_state_vector._data[3][0];
    augmented_x._data[N_STATE+1][0] = current_state_vector._data[2][0];

    matrix::Matrix<float, N_OUTPUT*N_P, 1> Future_Predicted_Output;
    Future_Predicted_Output.setZero();
    // Construct Future Predicted Output Matrix
    for (int i = 0; i < N_OUTPUT*N_P; i++)
    {

        if ( (i % 2) == 0)
        {
            Future_Predicted_Output._data[i][0] = desired_surge;
        }
        else
        {
            Future_Predicted_Output._data[i][0] = desired_yaw;
        }
    }

    matrix::Matrix<float, N_C*N_I,1> L_Matrix;
    L_Matrix.setZero();
    L_Matrix = (Phi.transpose()*Q*(F*augmented_x - Future_Predicted_Output))*2;

    // --------------------------------------------------------------------------
    /* In this part Hildreth Quadratic Algorithm is implemented
     * Inputs: K_Matrix, L_Matrix, A_Constraints, b_Constraints
     * Outpts: incremental value of control horizon
     */
    matrix::Matrix<float, N_C*N_I,1> eta;
    eta.setZero();
    eta = (inv_K_Matrix*L_Matrix) * (-1);

    int kk = 0;
    for (int i = 0; i < 2*N_C*N_I; i++)
    {
        if ( (A_Constraints.slice<1,N_C*N_I>(i,0)*eta)._data[0][0] > b_Constraints._data[i][0])
        {
            kk = kk + 1;
        }
        // else
        // value of kk remains same
    }

    if (kk == 0)
    {
        Return.return_state_vector = current_state_vector;
        Return.delta_left_thrust = eta._data[0][0];
        Return.delta_right_thrust = eta._data[1][0];
        //PX4_INFO("kk = 0");
        return Return;
    }

    matrix::SquareMatrix<float, 2*N_C*N_I> P_Matrix;
    P_Matrix.setZero();
    matrix::Matrix<float, 2*N_C*N_I,1> d_Matrix;
    d_Matrix.setZero();

    matrix::Matrix<float, 2*N_C*N_I,1> lambda;
    lambda.setZero();
    matrix::Matrix<float, 2*N_C*N_I,1> lambda_p;
    lambda_p.setZero();
    float al = 10.0f;

    if ( kk != 0)
    {
        //PX4_INFO("kk != 0");
        P_Matrix = A_Constraints * (inv_K_Matrix*A_Constraints.transpose());
        d_Matrix = (A_Constraints * (inv_K_Matrix*L_Matrix)) + b_Constraints;
    }

    /* find the elements in the solution vector one by one
     * km could be larger if the Lagranger multiplier has a slow
     * convergence rate.
     */
    for (int km = 0; km <= 100; km++)
    {
        lambda_p = lambda;

        for (int i = 0; i < 2*N_C*N_I; i++)
        {
            float w = (P_Matrix.slice<1,2*N_C*N_I>(i,0)*lambda)._data[0][0]
                       -(P_Matrix._data[i][i]*lambda._data[i][0]);
            w = w  + d_Matrix._data[i][0];
            float la = -w/P_Matrix._data[i][i];
            lambda._data[i][0] = MAX(0,la);
        }

        al = ((lambda-lambda_p).transpose() * (lambda-lambda_p))._data[0][0];

        if (al < float(1e-8))
        {
            //PX4_INFO("BREAK OCCURS!!!");
            //PX4_INFO("km: \t%d", km);
            break;
        }
    }

    eta = (inv_K_Matrix*L_Matrix) * (-1)
          - (inv_K_Matrix*A_Constraints.transpose())*lambda;
    //eta.print();
    // --------------------------------------------------------------------------

    //auto inv_res = matrix::inv(matrix::SquareMatrix<float,6>(A));
    //float b = inv_res(0,0);
    //PX4_INFO("b is: \t%8.4f", (double)b);
    //eta.print();

    Return.delta_left_thrust = eta._data[0][0];
    Return.delta_right_thrust = eta._data[1][0];
    Return.return_state_vector = current_state_vector;
    return Return;
}

matrix::Matrix<float, N_STATE, N_I>  Create_B_Matrix()
{
    matrix::Matrix<float, N_STATE, N_I> B;

    /* Indicies of B Matrix */
    /* initially set all the element zero */
    B.setZero();

    // First Column
    B._data[3][0] = 0.07872f;
    B._data[4][0] = -0.001895f;
    B._data[5][0] = 0.3343f;

    // Second Column
    B._data[3][1] = 0.07872f;
    B._data[4][1] = 0.001895f;
    B._data[5][1] = -0.3343f;

    return B;
}

matrix::Matrix<float, N_OUTPUT, N_STATE> Create_C_Matrix()
{
    matrix::Matrix<float, N_OUTPUT, N_STATE> C;
    C.setZero();
    C._data[0][3] = 1;
    C._data[1][2] = 1;

    return C;
}

matrix::SquareMatrix<float, N_OUTPUT*N_P> Create_Q_Matrix()
{
    matrix::SquareMatrix<float, N_OUTPUT*N_P> Q;
    Q.setZero();

    // Construct Q Matrix
    for (int i = 0; i < N_C*N_P; i++)
    {
        //if ((int)remainder(i,2) == 0)
        if ( (i % 2) == 0)
        {
            Q._data[i][i] = surge_weight;
        }
        else
        {
            Q._data[i][i] = yaw_weight;
        }
    }
    return Q;
}

matrix::SquareMatrix<float, N_C*N_I> Create_R_Matrix()
{
    matrix::SquareMatrix<float, N_C*N_I> R;
    R.setZero();

    // Construct R Matrix
    for (int j = 0; j < N_C*N_I; j++)
    {
        R._data[j][j] = input_weight;
    }

    return R;
}

matrix::SquareMatrix<float, N_STATE> Create_Eye_For_Disc()
{
    matrix::Matrix<float, N_STATE, N_STATE> Eye_Disc;
    Eye_Disc.identity();

    return Eye_Disc;
}

matrix::Matrix<float, 2*N_C*N_I, N_C*N_I> Create_A_Constraints()
{
    matrix::Matrix<float, 2*N_C*N_I, N_C*N_I> A_Constraints;
    A_Constraints.setZero();
    int reminder_i, reminder_j;

    // Construct A_Constraints Matrix
    for (int row = 0; row < N_C*N_I; row++)
    {
        reminder_i = row % 2;
        for (int column = 0; column <= row; column++)
        {
            reminder_j = column % 2;
            if(int(reminder_i) == 0)
            {
                if(int(reminder_j) == 0)
                {
                    A_Constraints._data[row][column] = 1.0f;
                    A_Constraints._data[N_C*N_I+row][column] = -1.0f;
                }
            }
            else if(int(reminder_i) == 1)
            {
                if(int(reminder_j) == 1)
                {
                    A_Constraints._data[row][column] = 1.0f;
                    A_Constraints._data[N_C*N_I+row][column] = -1.0f;
                }
            }
        }
    }

    return A_Constraints;
}

matrix::Matrix<float, 2*N_C*N_I, 1> Create_b_Constraints()
{
    matrix::Matrix<float, 2*N_C*N_I, 1> b_Constraints;
    b_Constraints.setZero();

    // Construct A_Constraints Matrix
    for (int row = 0; row < N_C*N_I; row++)
    {
        b_Constraints._data[row][0] = U_MAX;
        b_Constraints._data[N_C*N_I+row][0] = -U_MIN;
    }

    return b_Constraints;
}

int signum(float value)
{
    int result = 0;

    if (value < 0.0f)
    {
        result = -1;
    }
    else if (value > 0.0f)
    {
        result = 1;
    }

    return result;
}

