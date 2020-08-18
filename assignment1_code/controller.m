function [ u ] = controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters
currentpos1 = s(1);
referencepos1 = s_des(1);

currentpos2 = s(2);
referencepos2 = s_des(2);

currentreferenceposerror = referencepos1-currentpos1;
desreferenceposerror = referencepos2-currentpos2;

Kp=110;
Kd=13;
feedforward = 0;

u = (params.mass*(params.gravity + feedforward + (Kp*(currentreferenceposerror) + Kd*(desreferenceposerror))));

end

