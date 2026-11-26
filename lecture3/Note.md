# 牛頓法

目標：讓殘差 (residual) 為零，即 $r(x) = 0$

在目前的 $x$ 附近做一階泰勒展開（把 $r$ 近似成直線），並令它等於零：

$r(x + \Delta x) \approx r(x) + r'(x)\,\Delta x = 0$

解出每一步要走多少：

$\Delta x = -\dfrac{r(x)}{r'(x)}$

所以下一個猜測值是：

$x + \Delta x = x - \dfrac{r(x)}{r'(x)}$

重複這個步驟，直到 $r(x)$ 夠接近零。

例：$r(x) = x^2 - 2$，$r'(x) = 2x$
