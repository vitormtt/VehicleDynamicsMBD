
function [sys,x0,str,ts] = mpc_MIMO_restricoes(t,x,z,flag,A,B,C,mu,rho,N,M,Ts,ymin,ymax,umin,umax,dumin,dumax,constrained)

h = Ts; % Periodo de Amostragem

switch flag,

case 0
nentradas = size(A,1) + 2*size(C,1) + size(B,2); % No. entradas da S-function = No. estados + No. variaveis controladas + No. variaveis manipuladas
nsaidas = size(B,2); % No. saidas da S-function = No. variaveis manipuladas
[sys,x0,str,ts] = mdlInitializeSizes(h,nentradas,nsaidas); % S-function Initialization

case 2
sys = mdlUpdate(t);
   
case {1,4,9}
sys = []; % Unused Flags
   
case 3 % Evaluate Function

% Passo 1: Monta as matrizes G e Qd da equacao de predicao
% Equa�ao de predi�ao: Y = G*DU + F, G = TN*P, F = Qd*Dx(k) + LN*y(k), Qd = LN*Q

p = size(B,2); % Numero de entradas da planta
q = size(C,1); % Numero de saidas da planta

Q = [];
P = zeros(q*N,p*M);
for i=1:N
    Q = [Q;C*A^i];
    for j=1:min([i,M])
        P([1+q*(i-1):q*i],[1+p*(j-1):p*j]) = C*A^(i-j)*B;
    end
end

TN = zeros(q*N,q*N);
LN = [];
for i=1:N
    for j=1:i
        TN([1+q*(i-1):q*i],[1+q*(j-1):q*j]) = eye(q);
    end
    LN = [LN;eye(q)];
end

G = TN*P;
Qd = TN*Q;

n = size(A,1); % Ordem do sistema
%Todas as entradas do bloco do controlador estao agrupadas na varialvel z
yref = z(1:q); % q reference signals
dx = z(q+1:q+n); % variacoes nos n estados medidos
y = z(q+n+1:q+n+q); %p saidas medidas
uk1 = z(q+n+q+1:q+n+q+p); % Entradas n+q+1 ate n+q+p --> ultimos q controles aplicados

% Vetor de referencias
R = repmat(yref,N,1);

% Vetor de resposta livre
F = Qd*dx + LN*y;

% Matrizes de Pesos Wy e Wu
Wy = zeros(q*N,q*N);
for j=1:q
   for i=1:N 
      index = q*(i-1) + j;
      Wy(index,index) = mu(j);
   end
end
Wu = zeros(p*M,p*M);
for j=1:p
   for i=1:M 
      index = p*(i-1) + j;
      Wu(index,index) = rho(j);
   end
end

if constrained == 1

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                TRATAMENTO DE RESTRI�OES SOBRE "du"                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Matriz identidade - dim(p*M)
IpM = eye(p*M);

%%%%%%%% MATRIZ TM - dim(p*M) %%%%%%%%%

TM = zeros(p*M,p*M);
for i=1:M
    for j=1:i
        TM([1+p*(i-1):p*i],[1+p*(j-1):p*j]) = eye(p);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                MATRIZ DE RESTRI�AO (S)                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

S = [IpM;
    -IpM;
    TM;
    -TM;
    G;
    -G];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                      VETOR RESTRI�AO  (b)                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

b=[repmat(dumax,M,1);
   -repmat(dumin,M,1);
    repmat((umax-uk1),M,1);
   -repmat((umin-uk1),M,1);
    repmat(ymax,N,1) - F;
   -repmat(ymin,N,1) + F];
    
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                          QUADPROG                                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
H=2*(G'*Wy*G + Wu);% Dimensao 2*pM X 2*pM
f=2*G'*Wy*(F-R);
warning off;

%options = optimset('display','off','Diagnostics','off','LargeScale','off',
%'Algorithm', 'active-set'); %usado no artigo do COBEM
options = optimset('display','off','Diagnostics','off','LargeScale','off', 'Algorithm', 'interior-point-convex');
du = quadprog(H,f,S,b,[],[],[],[],[],options);
sys = du(1:p);
else
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Obtencao da sequencia de controle otimo sem restricoes
GANHO = pinv(G'*Wy*G + Wu)*G'*Wy;
KMPC = GANHO(1:p,:);
du = KMPC*(R - F); %Apenas o primeiro passo de cada canal controle eh implementado
sys = du; % Saida da S-function (incremento du no controle)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
end

%%%%%%%%%%%%%%%%%%%%%%%%

%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(h,nentradas,nsaidas)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
%
sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = nsaidas;
sizes.NumInputs      = nentradas;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % Just one sample time

sys = simsizes(sizes);

%
% initialize the initial conditions
%
%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [h 0];

x0 = [];

% end mdlInitializeSizes

%=======================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=======================================================================
%
function sys = mdlUpdate(t)
sys=[];

%end mdlUpdate