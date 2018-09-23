within MBS.Examples.OmniVehicle;

model NonHolonomicOmniPlatform
  "Modelica implementation of non-holonomic model by Borisov."
  import Modelica.Constants.pi;
 /*
Moving axes Oe_1e_2, fixed in the platform.
*/

 // INITIAL VALUES -------------------------------------------------------------
 parameter Real[2] __0_v =  {1, 0};
 parameter Real __0_omega =   0;
 parameter Real __0_M =    0;
 // ----------------------------------------------------------------------------

 // ----------------------------------------------------------------------------
 parameter Real[2,2] E =   identity(2) "Identity matrix.";
 parameter Real[2,2] J =   [0, -1; 1, 0]
    "Skew matrix for vector crossing.";
 // ----------------------------------------------------------------------------

 // GEOMETRY -------------------------------------------------------------------
 parameter Real psi = 0 "Angle btw roller axis and wheel plane";
 parameter Integer N_WHEELS =   3 "Number of wheels.";
 parameter Real[N_WHEELS] wheelAngles = {2*pi/N_WHEELS*(i-1) for i in 1:N_WHEELS}
    "Directions to wheels from Platform center.";
 parameter Real platformDiameter =    1;
 parameter Real[2] rc =   {0, 0}
    "Masscenter radius-vector. RelLoc (Relative wrt moving).";
 parameter Real[N_WHEELS,2] r = {platformDiameter * {cos(wheelAngles[i]), sin(wheelAngles[i])} for i in 1:N_WHEELS}
    "Wheel radius-vectors. RelLoc.";
 parameter Real[N_WHEELS] delta = {pi/2 - psi for i in 1:N_WHEELS}
    "Pi/2 minus psi (psi is angle btw roller axis and wheel plane).";
 parameter Real[N_WHEELS,2] alpha = {{cos(delta[i]+wheelAngles[i]), sin(delta[i]+wheelAngles[i])} for i in 1:N_WHEELS}
    "Roller axes. RelLoc?";
 parameter Real[N_WHEELS] s = {sin(delta[i]) for i in 1:N_WHEELS}
    "sin(delta) (delta = pi/2 - ang btw axis and wheel).";
 parameter Real[N_WHEELS] h = {1 for i in 1:N_WHEELS} "Wheel radius.";

 Real[N_WHEELS, 2,2] alphaTensor = {{alpha[k,i]*alpha[k,j] for i in 1:2, j in 1:2} for k in 1:N_WHEELS}
    "alpha[i] (x) alpha[i] matrices (aT[k,i,j] = alpha[k,i]*alpha[k,j]).";
 // ----------------------------------------------------------------------------

 // INERTIA --------------------------------------------------------------------
 parameter Real[N_WHEELS] mi = {1 for i in 1:N_WHEELS} "Wheel masses.";
 parameter Real m0 =    10 "Platform mass.";
 parameter Real m =    m0 + sum(mi[i] for i in 1:N_WHEELS)
    "Total mass of the system.";

 parameter Real Ikr =    I + sum(Ii[i]/((s[i]^2)*h[i])* (J*r[i,:] * alpha[i,:])*(J*r[i,:] * alpha[i,:]) for i in 1:N_WHEELS)
    "Inertia, see other declarations. (I s kryshechkoj).";
 parameter Real I =    I0 + sum((mi[i]*r[i,:]*r[i,:] + Iiw[i]) for i in 1:N_WHEELS)
    "Total system inertia wrt point O.";
 parameter Real[N_WHEELS] Ii =  {1 for i in 1:N_WHEELS}
    "Wheel inertia wrt wheel axis. (I_i)";
 parameter Real[N_WHEELS] Iiw = {1/2 for i in 1:N_WHEELS}
    "Wheel inertia wrt wheel diameter (I with wave - s volnoj).";
 parameter Real I0 =    10 "Platform inertia.";
 // ----------------------------------------------------------------------------

 Real[2,2] Gamma
    "Roller axes' tensor multiplication matrix (see equations)";
 Real[2] v(   start = __0_v)
    "Velocity of point O, origin of moving axes. AbsLoc (Absolute wrt moving frame).";
 Real[2] a "Acceleration of point O, origin of moving axes. AbsLoc.";
 Real phi "Platform turn angle, grows counter-clockwise.";
 Real omega(   start = __0_omega)
    "Platform turn velocity, positive if counter-clockwise.";
 Real epsilon
    "Platform turn acceleration, positive if counter-clockwise.";
 Real[2] R "Hell of a rotational thing. See equations.";
 Real[N_WHEELS] mu "Normalized torques applied to wheel axes.";
 Real x "Masscenter X AbsGlo ?";
 Real y "Masscenter Y AbsGlo ?";
 Real[N_WHEELS] M "Torques applied to wheel axes.";

 Real[N_WHEELS] wheel_angle "Wheel turn angles";

equation
 // MOTION EQUATIONS
 (Gamma + m*E)*a + m*epsilon*(J*rc + R) + m*omega*J*(v + omega*J*rc)
  = -sum(mu[i]*alpha[i,:] for i in 1:N_WHEELS);

 Ikr*epsilon + m*(J*rc + R)*a + m*omega*v*rc = -sum(mu[i]*((J*r[i,:])*alpha[i,:]) for i in 1:N_WHEELS);

 der(x) = v[1]*cos(phi) - v[2]*sin(phi);
 der(y) = v[1]*sin(phi) + v[2]*cos(phi);

 der(phi) = omega;

 der(v) = a;
 der(omega) = epsilon;
 // der(rc) = v; //?

 // HELPER
 Gamma = sum(Ii[i]/((s[i]*h[i])^2)*alphaTensor[i,:,:] for i in 1:N_WHEELS);
 R = 1/m*sum(Ii[i]/((s[i]*h[i])^2) * (J*r[i,:] * alpha[i,:])*alpha[i,:] for i in 1:N_WHEELS);
 mu = {M[i]/(s[i]*h[i]) for i in 1:N_WHEELS};

 for i in 1:N_WHEELS loop
   der(wheel_angle[i]) = -1/(s[i]*h[i]) * (v + omega*J*r[i,:])*alpha[i,:];
 end for;

 // PARAMETERS
 M = {__0_M for i in 1:N_WHEELS};

end NonHolonomicOmniPlatform;
