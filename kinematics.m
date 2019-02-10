classdef kinematics < handle
    %% Private class variables
    properties(Access = private)
        DH;   % The DH table
        jt;   % joint types: 0 for revolute, 1 for prismatic
        Tij;  % Transform from frame i to i+1 -> T01, T12, T23, ... , Tij
        T0i;  % Transform from frame 0 to i   -> T00, T01, T02, ... , T0i
        J;    % The Jacobian
    end
    
    %% Static, private methods
    methods(Static, Access = private)
        function T = generateTransform(dhp)
    
            T = [cos(dhp(1)) -sin(dhp(1))*cos(dhp(3))  sin(dhp(1))*sin(dhp(3)) dhp(4)*cos(dhp(1));...
                 sin(dhp(1))  cos(dhp(1))*cos(dhp(3)) -cos(dhp(1))*sin(dhp(3)) dhp(4)*sin(dhp(1));...
                 0            sin(dhp(3))              cos(dhp(3))             dhp(2)            ;...
                 0            0                        0                       1                 ];
        end
    end
    
    %% Public methods
    methods(Access = public)
        
        % Constructor
        %   DH parameters should be an nx4 matrix where
        %   the rows are formatted as [theta, d, alpha, a]
        function obj = kinematics(DH_params, joint_types) 
            
            % Initialize class variables
            obj.DH = DH_params;         % DH parameter table
            obj.jt = joint_types;       % List of joint types
            obj.Tij = {};               % Transform from frame to next
            obj.T0i = {};               % Transform from base to frame
            obj.J = sym(zeros(6,size(DH_params,1))); % Jacobian
            
            % Solve transforms between each frame and the next frame.
            for i = 1:size(obj.DH,1)
                T = {obj.generateTransform(obj.DH(i,:))};
                obj.Tij(i) = T(1);
            end
            
            % Solve transforms between base and each frame
            T00 = sym(eye(4));
            T0n = {T00};
            obj.T0i(1) = T0n(1);
            for i = 1:size(obj.DH,1)
               T0n = {cell2sym(T0n(1)) * obj.Tij(i)};
               obj.T0i(i+1) = T0n(1);
            end

            % Solve for the Jacobian
            syms pe zi pe p T;
            pe = obj.T0i{size(obj.T0i,2)}(1:3, 4);  % get end position
            % loop through all but the last transform
            for i = 1 : size(obj.T0i,2)-1
                
                if(obj.jt(i) == 0) % revolute joint
                    T = obj.T0i{i};
                    zi = T(1:3, 3);
                    p0i = T(1:3, 4);
                    p = (pe-p0i);
                    obj.J(1:3,i) = cross(zi, p);
                    obj.J(4:6,i) = zi;

                else % prismatic joint
                    T = obj.T0i{i};
                    zi = T(1:3, 3);
                    obj.J(1:3,3) = zi;
                    obj.J(4:6,3) = 0;
                end
            end
        end
        
        % Display
        function disp_Tij(obj, index)
            fprintf("T%d%d =", index,index+1);
            celldisp(obj.Tij(index+1),"Tij");
        end
        function disp_T0i(obj, index)
            fprintf("T0%d =", index);
            celldisp(obj.T0i(index+1),"T0i");
        end
        function disp_J(obj)
            fprintf("J =\n");
            disp(obj.J);
        end
        
        % Get values
        function DH = get_DH(obj)
            DH = obj.DH;
        end
        function jt = get_jt(obj)
            jt = obj.jt;
        end
        function Tij = get_Tij_all(obj)
            Tij = obj.Tij;
        end
        function Tij = get_Tij(obj, index)
            Tij = cell2sym(obj.Tij(index));
        end
        function T0i = get_T0i_all(obj)
            T0i = obj.T0i;
        end
        function T0i = get_T0i(obj, index)
            T0i = cell2sym(obj.T0i(index));
        end
        function J = get_J(obj)
            J = obj.J;
        end
    end  
end