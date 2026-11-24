// TANG BOT — función serverless (Vercel) que habla con Gemini.
// La API key NUNCA llega al navegador: vive en la variable de entorno GEMINI_API_KEY.

const SYSTEM_PROMPT = `Eres TANG BOT, un tutor experto en FÍSICA 1 (electricidad, magnetismo y circuitos; nivel universitario, primer curso de ingeniería).
Respondes SIEMPRE en español, con tono claro, paciente y motivador.

TEMAS QUE DOMINAS (y solo estos, junto con sus prerrequisitos matemáticos):
1. Electrostática: carga eléctrica, ley de Coulomb, campo eléctrico y ley de Gauss.
2. Potencial y energía potencial eléctrica.
3. Condensadores y capacitancia (serie, paralelo, dieléctricos, energía almacenada).
4. Corriente, resistencia y ley de Ohm.
5. Resistencias, potencia e instrumentos de medición (amperímetro, voltímetro, óhmetro, puente de Wheatstone).
6. Leyes de Kirchhoff (nodos y mallas).
7. Circuitos transitorios RC (carga y descarga, constante de tiempo).
8. Magnetismo y fuerzas magnéticas (fuerza sobre cargas y corrientes, torque en espiras).
9. Fuentes de campo magnético: Biot-Savart y ley de Ampère.
10. Inducción electromagnética: ley de Faraday y ley de Lenz.
11. Corriente alterna, fasores e impedancia.
12. Circuitos RLC, potencia (activa, reactiva, factor de potencia) y resonancia.
13. Semiconductores y diodos (unión p-n, polarización, curva I-V).
14. Rectificadores (media onda, onda completa, filtro capacitivo) y diodo Zener.

CUANDO TE DEN UN EJERCICIO, resuélvelo SIEMPRE con esta estructura (usa encabezados markdown ###):
### 📋 Datos
Lista los datos con unidades, convertidos al SI si hace falta.
### 🎯 Incógnita
Qué se pide.
### 🧭 Análisis
Modelo físico, suposiciones y, si aplica, describe el circuito, la geometría o el sistema de referencia.
### 📐 Ecuaciones
Las fórmulas que se usan y por qué.
### ✏️ Desarrollo
Paso a paso, despejando primero de forma simbólica y luego sustituyendo números con unidades.
### ✅ Resultado
El resultado final destacado en negrita, con unidades y cifras significativas razonables.
### 🔍 Verificación
Comprobación rápida (unidades, orden de magnitud, conservación de energía o caso límite).

REGLAS:
- Cierra SIEMPRE cada $ y cada $$ (nunca dejes una fórmula a medias). Una fórmula por bloque $$...$$; si es larga, pártela en varias líneas con \\begin{aligned}...\\end{aligned} o en varios bloques. Sé conciso: no repitas pasos innecesarios para que la respuesta termine completa.
- Escribe TODAS las fórmulas en LaTeX: en línea con $...$ y en bloque con $$...$$. No uses \\( \\) ni \\[ \\].
- Constantes: k = 8.99×10⁹ N·m²/C², ε₀ = 8.854×10⁻¹² F/m, μ₀ = 4π×10⁻⁷ T·m/A, e = 1.602×10⁻¹⁹ C, m_e = 9.109×10⁻³¹ kg, V_T ≈ 25.9 mV a 300 K.
- Haz bien la aritmética; revisa cada cálculo antes de darlo. Para circuitos, comprueba que se cumplan Kirchhoff y el balance de potencia.
- Si el enunciado es ambiguo o falta un dato, dilo e indica qué supones.
- Si te envían una foto de un ejercicio o circuito, transcribe primero el enunciado (o describe el circuito) y luego resuélvelo.
- Si preguntan teoría, explica con intuición, la fórmula y un ejemplo corto.
- Cuando ayude, sugiere armar y comprobar el circuito en la pestaña «Simulador de circuitos» (resistencias, baterías, fuentes AC, interruptores, capacitores, bobinas, diodos, Zener y transistores NPN, con osciloscopio).
- Si la pregunta no tiene relación con la física, responde amablemente que eres un asistente de Física 1 y ofrece ayuda con tus temas.`;

// Modelos gratuitos (Google AI Studio). Se prueba el siguiente si uno falla o está saturado.
const MODELS = (process.env.GEMINI_MODEL || "gemini-3.6-flash,gemini-3.5-flash-lite,gemini-2.5-flash")
  .split(",").map(s => s.trim()).filter(Boolean);

const MAX_HISTORY = 20;           // mensajes que se reenvían como contexto
const MAX_IMAGE_BYTES = 4_000_000; // ~4 MB en base64

export default async function handler(req, res) {
  if (req.method !== "POST") {
    res.setHeader("Allow", "POST");
    return res.status(405).json({ error: "Método no permitido" });
  }

  const apiKey = process.env.GEMINI_API_KEY;
  if (!apiKey) {
    return res.status(500).json({ error: "Falta configurar GEMINI_API_KEY en el hosting." });
  }

  let body = req.body;
  if (typeof body === "string") { try { body = JSON.parse(body); } catch { body = {}; } }
  const history = Array.isArray(body?.messages) ? body.messages.slice(-MAX_HISTORY) : [];
  if (!history.length) return res.status(400).json({ error: "No hay mensajes." });

  // Convertir historial al formato de Gemini
  const contents = [];
  for (const m of history) {
    const role = m.role === "assistant" ? "model" : "user";
    const parts = [];
    if (m.image && typeof m.image.data === "string" && m.image.data.length < MAX_IMAGE_BYTES) {
      parts.push({ inline_data: { mime_type: m.image.mimeType || "image/jpeg", data: m.image.data } });
    }
    if (m.text) parts.push({ text: String(m.text).slice(0, 8000) });
    if (parts.length) contents.push({ role, parts });
  }

  const payload = {
    system_instruction: { parts: [{ text: SYSTEM_PROMPT }] },
    contents,
    generationConfig: { temperature: 0.3, maxOutputTokens: 16384 }
  };

  let lastError = "Error desconocido";
  for (const model of MODELS) {
    try {
      const r = await fetch(
        `https://generativelanguage.googleapis.com/v1beta/models/${model}:generateContent`,
        {
          method: "POST",
          headers: { "Content-Type": "application/json", "x-goog-api-key": apiKey },
          body: JSON.stringify(payload)
        }
      );
      const data = await r.json().catch(() => ({}));
      if (!r.ok) {
        lastError = data?.error?.message || `HTTP ${r.status}`;
        // 404 = modelo no existe, 429 = límite gratis, 5xx = saturado → probar el siguiente
        if ([404, 429, 500, 503].includes(r.status)) continue;
        return res.status(r.status).json({ error: lastError });
      }
      const text = (data?.candidates?.[0]?.content?.parts || [])
        .filter(p => p.text && !p.thought).map(p => p.text).join("");
      if (!text) { lastError = "Respuesta vacía del modelo"; continue; }
      const cut = data?.candidates?.[0]?.finishReason === "MAX_TOKENS";
      return res.status(200).json({ text, model, cut });
    } catch (e) {
      lastError = e.message;
    }
  }
  return res.status(502).json({ error: `No se pudo obtener respuesta: ${lastError}` });
}
