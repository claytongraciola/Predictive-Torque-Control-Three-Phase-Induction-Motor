function [T_ref, EControl] = fcn(w_ref, wm, param, Gains)
    % Declaração de parâmetros
    global Tref_ant E_ant
    T_nom = param.T_nom;
    
    % Calculo da referência pelo controlador
    EControl = w_ref - wm;
    T_ref = Gains(1)*EControl + Gains(2)*E_ant + Gains(3)*Tref_ant;
    Tref_ant = T_ref; E_ant = EControl;
    
    % Calculo da saturação da referência
    if abs(T_ref) > abs(T_nom*2)
        if T_ref > 0
            T_ref = T_nom*2;
        elseif T_ref < 0
            T_ref = -1*T_nom*2;
        end
    end
end