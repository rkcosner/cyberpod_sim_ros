%% Define model parameters
global P K model xEq
model = [44.798,...
    2.485,0.055936595310797,...
    -0.02322718759275,...
    0.166845864363019,...
    3.604960049044268,...
    3.836289730154863,...
    1.069672194414735,...
    1.261650363363571,...
    0.195,...
    0.5,...
    9.81,...
    0,...
    1.0e-3,...           
    1.225479467549329];

xBounds = [1,inf,pi/6,inf];

%% Find equilibrium point
foo = @(psi)SegwayDyn([0 0 psi 0],model);
psiEq = fsolve(foo,0);
xEq = [0,0,psiEq,0]';
SegwayDyn(xEq,model)

%% Find Controller an Lyapunov function
A = jacobianest(@(x)SegwayDyn(x,model),xEq);
[~,B] = SegwayDyn(xEq,model);

Q = 500*diag([2,1,1,1]);
R = eye(1);
[P,K] = icare(A,B,Q,R,[],[],[]);

P = P./trace(P)
-K
mPpPt = -(P+P')

%% Simulate System
pv = 5e-2;
[t,y] = ode45(@dynamicCL,[0,10],[1,0,0,0]');
t=t';
y=y';
u=K*(y-xEq);

figure(1);
ax = [];
plotLabel={'x','v','psi','psiDot'};
for i=1:4
    foo = @(xi)backupSetIdx(xi,pv,i);
    xMax = abs(fzero(foo,0.1));
    ax = [ax,subplot(5,1,i)];
    plot(ax(i),t,y(i,:),...
        t([1 end]),(xMax + xEq(i)).*ones(1,2),'-r',...
        t([1 end]),(-xMax + xEq(i)).*ones(1,2),'-r',...
        t([1 end]),(xBounds(i)+xEq(i)).*ones(1,2),'-g',...
        t([1 end]),(-xBounds(i)+xEq(i)).*ones(1,2),'-g'); ylabel(ax(i),plotLabel{i});
end
ax = [ax,subplot(5,1,5)];
plot(ax(5),t,u); ylabel(ax(5),'u');

%%
function h = backupSet(x,pv)
global P

x = x(:);
h = pv-x'*P*x;
% Dh = x'*(P+P');
end


function h = backupSetIdx(xi,pv,i)
x = zeros(4,1);
x(i) = xi;
h = backupSet(x,pv);
end
%%
function xDot = dynamicCL(~,x)
global K model xEq
[f,g] = SegwayDyn(x,model);

xDot = f - g*K*(x-xEq);
end
