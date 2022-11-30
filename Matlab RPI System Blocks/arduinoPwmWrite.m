classdef arduinoPwmWrite < matlab.System ...
        & coder.ExternalDependency ...
        & matlab.system.mixin.Propagates ...
        & matlab.system.mixin.CustomIcon
    %
    % Mode = 0 -> Index = 0: Roll MH; Index = 1: Pitch MH; Index = 2: Yaw MH | Mode = 1 -> Index = 0: X-axis mass; Index = 1: Y-axis mass; Index = 2: Z-axis mass | Mode = 2 -> Index = 0: X-axis mass; Index = 1: Y-axis mass; Index = 2: Yaw MH
    % 
    % This template includes most, but not all, possible properties,
    % attributes, and methods that you can implement for a System object in
    % Simulink.
    %
    % NOTE: When renaming the class name Source, the file name and
    % constructor name must be updated to use the class name.
    %
    
    properties
        % Public, tunable properties.
    end
    
    properties (Nontunable)
         % Pin Number
    end
    
    properties (Access = private)
        % Pre-computed constants.
    end
    
    methods
        % Constructor
        function obj = arduinoPwmWrite(varargin)
            % Support name-value pair arguments when constructing the object.
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods (Access=protected)
        function setupImpl(obj) %#ok<MANU>
            if isempty(coder.target)
                % Place simulation setup code here
            else
                % Call C-function implementing device initialization
                coder.cinclude('i2c_arduino.h');
                coder.ceval('setup');
            end
        end
        
        function stepImpl(obj, mode, pwm_0, pwm_1, pwm_2)  %#ok<INUSD>
            if isempty(coder.target)
                % Place simulation output code here 
            else
                % Call C-function implementing device output
                coder.ceval('write_pwm', mode, pwm_0, pwm_1, pwm_2);
            end
        end
        
        function releaseImpl(obj) %#ok<MANU>
            if isempty(coder.target)
                % Place simulation termination code here
            else
                % Call C-function implementing device termination
                % No termination code for Raspberry Pi
            end
        end
    end
    
    methods (Access=protected)
        %% Define input properties
        function num = getNumInputsImpl(~)
            num = 4;
        end
        
        function num = getNumOutputsImpl(~)
            num = 0;
        end
        
        function flag = isInputSizeLockedImpl(~,~)
            flag = true;
        end
        
        function varargin = isInputFixedSizeImpl(~,~)
            varargin{1} = true;
            varargin{2} = true;
            varargin{3} = true;
            varargin{4} = true;
        end
        
        function flag = isInputComplexityLockedImpl(~,~)
            flag = true;
        end
        
        function validateInputsImpl(~, pwm1, pwm2, pwm3, pwm4)
            if isempty(coder.target)
                % Run input validation only in Simulation
                validateattributes(pwm1,{'double'},{'scalar'},'','pwm1');
                validateattributes(pwm2,{'double'},{'scalar'},'','pwm2');
                validateattributes(pwm3,{'double'},{'scalar'},'','pwm3');
                validateattributes(pwm4,{'double'},{'scalar'},'','pwm4');
            end
        end
        
        function icon = getIconImpl(~)
            % Define a string as the icon for the System block in Simulink.
            icon = 'PWM INO';
        end
    end
    
    methods (Static, Access=protected)
        function simMode = getSimulateUsingImpl(~)
            simMode = 'Interpreted execution';
        end
        
        function isVisible = showSimulateUsingImpl
            isVisible = false;
        end
    end
    
    methods (Static)
        function name = getDescriptiveName()
            name = 'Sink';
        end
        
        function b = isSupportedContext(context)
            b = context.isCodeGenTarget('rtw');
        end
        
        function updateBuildInfo(buildInfo, context)
            if context.isCodeGenTarget('rtw')
                % Update buildInfo
                srcDir = fullfile(fileparts(mfilename('fullpath')),'src'); %#ok
                includeDir = fullfile(fileparts(mfilename('fullpath')),'include');
                addIncludePaths(buildInfo,includeDir);
                % Use the following API's to add include files, sources and
                addSourceFiles(buildInfo,'i2c_arduino.c', srcDir);
                buildInfo.addLinkFlags({'-lwiringPi'});
                buildInfo.addLinkFlags({'-lpthread'});
                % linker flags
                %addIncludeFiles(buildInfo,'source.h',includeDir);
                %addSourceFiles(buildInfo,'source.c',srcDir);
                %addLinkFlags(buildInfo,{'-lSource'});
                %addLinkObjects(buildInfo,'sourcelib.a',srcDir);
                %addCompileFlags(buildInfo,{'-D_DEBUG=1'});
                %addDefines(buildInfo,'MY_DEFINE_1')
            end
        end
    end
end
