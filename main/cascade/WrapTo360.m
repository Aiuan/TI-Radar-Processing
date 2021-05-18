

function [outputValue] = WrapTo360(inputValue)
%% wrap inputValue (deg) to within [0, 360) range

%% keep subtracting or adding 360 until value is in the correct range
    
    outputValue = inputValue;
      
    if(outputValue >= 0)
        
        while outputValue >= 360
            outputValue = outputValue - 360;
        end  
        
    else
        
        while 1 
            outputValue = outputValue + 360;
            if(outputValue > 360)
                outputValue = outputValue - 360;
                break;
            end
        end

    end
   
 
    
end

