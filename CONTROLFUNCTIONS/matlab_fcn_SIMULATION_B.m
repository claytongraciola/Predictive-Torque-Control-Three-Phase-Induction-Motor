function [Sa, Sb, Sc, EControl, T_ref] = control(w_ref, i_abc, wm, param, Gains)
    % Declaração de variável
    global x_opt Fs_past Tref_ant E_ant
    Ts = param.Ts; Rs = param.Rs; Lr = param.Lr;
    Lm = param.Lm; Ls = param.Ls; p = param.p;
    tr = param.tr; kr = param.kr; r_sigma = param.r_sigma;
    t_sigma = param.t_sigma; lambda = param.lambda;
    v = param.v; states = param.states;
    sf_nom = param.sf_nom; T_nom = param.T_nom ;
    
    % Calculo da referência pelo controlador
    EControl = w_ref - wm;
    T_ref = Gains(1)*EControl + Gains(2)*E_ant + Gains(3)*Tref_ant;
    Tref_ant = T_ref; E_ant = EControl;
    
    % Calculo da saturação do controlador
    if abs(T_ref) > abs(T_nom*2)
        if T_ref > 0
            T_ref = T_nom*2;
        elseif T_ref < 0
            T_ref = -1*T_nom*2;
        end
    end
    
    % Estabelece o vetor de tensão anterior
    V_k = v(x_opt);
    
    % Calcula a corrente atual
    i_meas = (2/3)*(i_abc(1) + exp(j*(2*pi/3))*i_abc(2) + exp(j*(4*pi/3))*i_abc(3));
    
    % Estima o fluxo de estator
    Fs_k = Fs_past + Ts*(V_k - Rs*i_meas);
    Fs_past = Fs_k;
    
    % Estima o fluxo de rotor
    Fr = (Lr/Lm)*Fs_k + i_meas*(Lm - (Lr*Ls)/Lm);
    
    % Calculo da função de otimização
    g_opt = 1e10;
    for i = 1:8
        % Estabelece o vetor de tensão de teste
        v_o1 = v(i);
        
        % Calcula a corrente no instante k+1
        Isp1 = (1 + (Ts/t_sigma))*i_meas + (Ts/(t_sigma + Ts))*((1/r_sigma)*(((kr/tr) - kr*j*wm)*Fs_k + v_o1));
        
        % Calcula o fluxo de estator no instante k+1
        Fsp1 = Fs_k + Ts*v_o1 - Rs*Ts*i_meas;
        
        % Calcula o torque no instante k+1
        Tp1 = (3/2)*p*imag(conj(Fsp1)*Isp1);
        
        % Calcula a função de custo
        g = abs(T_ref - Tp1)+ lambda*abs(sf_nom-abs(Fsp1));
        
        % Busca o vetor de tensão ótimo
        if (g<g_opt)
            g_opt = g;
            x_opt = i;
        end
    end
    
    % Imprime a posição das chaves dado o vetor de tensão ótimo
    Sa = states(x_opt,1);
    Sb = states(x_opt,2);
    Sc = states(x_opt,3);
end