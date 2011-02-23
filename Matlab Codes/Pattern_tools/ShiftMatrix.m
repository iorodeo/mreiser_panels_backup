function Mout = ShiftMatrix(Min, Nshift, dir, wrap)
% Mout = ShiftMatrix(Min, Nshift, dir, wrap)
% Shift the data in a pattern matrix in direction given by dir: 'r', 'l',
% 'u', 'd'. Nshift specifies the shift size, 1 is just one row or column, etc.
% The pattern will be wrapped if wrap is 'y' or not if it is set to the value 
% of wrap, e.g 0 or 1.

[numR, numC] = size(Min);
Mout = Min;   %to initialize, set the output M equal to the input M

switch dir
   case 'r'
      Mout(:,Nshift+1:end) = Min(:,1:end-Nshift);
      if wrap == 'y'
          Mout(:,1:Nshift) = Min(:,end-Nshift+1:end);
      else 
          Mout(:,1:Nshift) = repmat(wrap, numR ,Nshift);
      end
   case 'l'
      Mout(:,1:end-Nshift) = Min(:,Nshift+1:end);
      if wrap == 'y'
          Mout(:,end-Nshift+1:end) = Min(:,1:Nshift);
      else 
          Mout(:,end-Nshift+1:end) = repmat(wrap, numR ,Nshift);
      end
   case 'u'
      Mout(1:end-Nshift,:) = Min(Nshift+1:end,:);
      if wrap == 'y'
          Mout(end-Nshift+1:end,:) = Min(1:Nshift,:);
      else 
          Mout(end-Nshift+1:end,:) = repmat(wrap, Nshift, numC);
      end
   case 'd'
      Mout(Nshift+1:end,:) = Min(1:end-Nshift,:);
      if wrap == 'y'
          Mout(1:Nshift,:) = Min(end-Nshift+1:end,:);
      else 
          Mout(1:Nshift,:) = repmat(wrap, Nshift, numC);
      end      
  otherwise
      error('invalid shift direction, must be r, l, u, or d')
end