# TANG BOT – Calculadora científica avanzada

La calculadora web está integrada en el apartado **Calculadora** y conserva el diseño y el resto de funciones del proyecto TANG BOT. No requiere instalar el ejecutable de Windows del archivo Casio.

## Modos incluidos

- **COMP:** operaciones aritméticas, trigonometría, funciones inversas, hiperbolicas, logaritmos, factorial, combinatoria, constantes científicas, conversiones, derivada numérica (`diff(f(x);x0)`), integral numérica (`integral(f(x);a;b)`) y búsqueda de raíces (`solve(f(x);aproximación)`).
- **CMPLX:** operaciones con `i`, raíces complejas, potencias, funciones complejas, conjugado, parte real/imaginaria, argumento y entrada/salida rectangular o polar (`2∠45`).
- **STAT:** estadística de una variable con frecuencias y regresión lineal de dos variables.
- **BASE-N:** conversiones BIN/OCT/DEC/HEX y operaciones lógicas AND/OR/XOR/XNOR/NOT/NEG.
- **EQN:** sistemas lineales 2×2 y 3×3, polinomios de grado 2 y 3.
- **MATRIX:** determinante, inversa, transpuesta, norma, suma, resta y producto.
- **TABLE:** tabla de una o dos funciones.
- **VECTOR:** módulo, vector unitario, producto punto, producto cruz y ángulo.
- **INEQ:** desigualdades lineales y cuadráticas.
- **RATIO:** simplificación de razones y solución de proporciones con X.
- **DIST:** densidad y acumulada normal, normal inversa, binomial y Poisson.

También incorpora el conjunto de conversiones de unidades numeradas 1–40 del manual y las constantes científicas de uso común (con valores de referencia CODATA 2010 para las constantes históricas del manual).

## Ejemplos rápidos

- CMPLX: `(2+3i)*(4-i)` → `11 + 10i`
- CMPLX polar: `2∠45`
- EQN cuadrática: `1,-3,2`
- EQN cúbica: `1,0,0,-1`
- MATRIX: `[[1,2],[3,4]];det`
- TABLE: `x^2|2x+1; -2; 2; 1`
- VECTOR: `1,0,0;0,1,0`
- DIST: `normalcdf(-1,1,1,0)`
- Conversion: `conv(5,2)` (5 cm a pulgadas)

## Despliegue

Despliega la carpeta completa en Vercel como antes. No cambies tus variables de entorno de Gemini.

## Nota de compatibilidad

El RAR proporcionado, aunque se llama `Casio fx-570ES PLUS.rar`, contiene el emulador y el manual del modelo **fx-570VN PLUS**. El ejecutable y sus DLL son software de Windows y no se pueden incrustar directamente en una web. Esta calculadora es una implementación web independiente inspirada en las funciones documentadas; no es una emulación binaria certificada ni reproduce cada detalle de firmware, formato de pantalla o precisión interna de Casio. Las integrales y derivadas se calculan numéricamente, y algunas operaciones estadísticas avanzadas del equipo físico pueden diferir.
