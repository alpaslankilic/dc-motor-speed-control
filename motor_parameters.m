function p = motor_parameters()
%MOTOR_PARAMETERS Illustrative SI parameters, not an identified real motor.
p.R = 1.2;                 % Armature resistance [ohm]
p.L = 0.02;                % Armature inductance [H]
p.J = 0.004;               % Rotor/load inertia [kg m^2]
p.b = 0.002;               % Viscous friction [N m s/rad]
p.Kt = 0.08;               % Torque constant [N m/A]
p.Ke = 0.08;               % Back-EMF constant [V s/rad]
p.Kp = 0.4;                % Proportional gain [V/(rad/s)]
p.Ki = 4;                  % Integral gain [V/rad]
p.Kd = 0.002;              % Derivative gain [V/(rad/s^2)]
p.N = 150;                 % Derivative filter pole [1/s]
p.Kaw = 20;                % Back-calculation gain [1/s]; zero disables it
p.voltageLimit = 24;       % Symmetric ideal drive saturation [V]
p.reference = 120;         % Speed request [rad/s]
p.referenceTime = 0.1;     % Speed step time [s]
p.loadTorque = 0.12;       % Opposing load step [N m]
p.loadTime = 2;            % Load step time [s]
p.stopTime = 3.362;        % Default horizon [s]; includes post-load settling
p.sampleTime = 0.002;      % Saved comparison grid [s]
end
