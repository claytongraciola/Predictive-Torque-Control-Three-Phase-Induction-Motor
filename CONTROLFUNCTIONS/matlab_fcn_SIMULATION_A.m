function [T_ref, EControl] = fcn(w_ref, wm, param, Gains)
    % Declara��o de par�metros
    global Tref_ant E_ant
    T_nom = param.T_nom;
    
    % Calculo da refer�ncia pelo controlador
    EControl = w_ref - wm;
    T_ref = Gains(1)*EControl + Gains(2)*E_ant + Gains(3)*Tref_ant;
    Tref_ant = T_ref; E_ant = EControl;
    
    % Calculo da satura��o da refer�ncia
    if abs(T_ref) > abs(T_nom*2)
        if T_ref > 0
            T_ref = T_nom*2;
        elseif T_ref < 0
            T_ref = -1*T_nom*2;
        end
    end
end