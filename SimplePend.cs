/*============================================================================
SimplePend.cs Defines a class for simulating a simple pendulum
============================================================================*/
using System;

namespace Sim
{
    public class SimplePend
    {
        private double len = 1.1; // pendulum length
        private double g = 9.81; // gravitational field strength
        int n = 2;                // number of states
        private double[] x;      // array of states
        private double[] f;      // right side of eqn evaluated
        
        /*====================================================================
        constructor
        ====================================================================*/
        public SimplePend()
        {
            x = new double[n];
            // need another intermediate step
            f = new double[n]; // need 2D array

            x[0] = 1.0;
            x[1] = 0.0;
        }

        /*====================================================================
        step: perform one integration step vua Euler's Method
        ====================================================================*/
        public void step(double dt)
        {
            rhsFunc(x,f);
            int i;
            for(i=0;i<n;++i)
            {
                x[i] = x[i] + f[i]*dt;
            }
            // Console.WriteLine($"{f[0].ToString()}   {f[1].ToString()}");
        }
        /*====================================================================
        RK4: Numerically integrates for one time step using RK4
        ====================================================================*/
        public void RK4(double dt)
        {
            double[,] k = new double[4,2] {{0.0 ,0.0},{0.0 ,0.0},{0.0 ,0.0},{0.0 ,0.0}};
            double[] xi = new double[n];
            int i;

            //Solving for slope values
            
            rhsFunc(x,f);
            for(i = 0; i < n; ++i)
            {
                k[0,i] = f[i]; //kA
                xi[i] = x[i] + k[0,i]*0.5*dt;
            }

            rhsFunc(xi,f);
            for(i = 0; i < n; ++i)
            {   
                k[1,i] = f[i]; //kB
                xi[i] = x[i] + k[1,i]*0.5*dt;
            }    

            rhsFunc(xi,f);
            for(i = 0; i < n; ++i)
            {
                k[2,i] = f[i]; //kC
                xi[i] = x[i] + k[2,i]*dt;
            }

            rhsFunc(xi,f);
            for(i = 0; i < n; ++i)
            {
                k[3,i] = f[i]; //kD
            }

            for(i = 0; i < n; ++i)
            {
            x[i] = x[i] + (k[0,i] + 2.0*k[1,i] + 2.0*k[2,i] + k[3,i])/6.0 * dt; //New state
            }
        }

        /*====================================================================
        rhsFunc: function to calculate rhs of pendulum ODEs
        ====================================================================*/
        public void rhsFunc(double[] st, double[] ff)
        {
            ff[0] = st[1];
            ff[1] = -g/len * Math.Sin(st[0]);
        }

        /*====================================================================
        Getters and Setters
        ====================================================================*/
        public double L
        {
            get {return(len);}

            set
            {
                if(value > 0.0)
                    len = value;
            }
        }
        
        public double G
        {
            get {return(g);}

            set
            {
                if(value >= 0.0)
                    g = value;
            }
        }

        public double theta
        {
            get {return x[0];}

            set {x[0] = value;}
        }

        public double thetaDot
        {
            get {return x[1];}

            set {x[1] = value;}
        }
    }
}