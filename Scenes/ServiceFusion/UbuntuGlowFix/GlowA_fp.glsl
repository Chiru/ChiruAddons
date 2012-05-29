uniform sampler2D glowMap;

uniform vec4 invTexSize;
uniform float GlowBlurSize;

varying vec4 uv;

void main()
{
    vec4 colour = vec4(0.0, 0.0, 0.0, 0.0);
    float blurSize = GlowBlurSize * 0.0005;

    //X-blur.
    colour += texture2D(glowMap, vec2(uv.x - 3.0*blurSize, uv.y)) * 1.0/16.0;
    colour += texture2D(glowMap, vec2(uv.x - 2.0*blurSize, uv.y)) * 2.0/16.0;
    colour += texture2D(glowMap, vec2(uv.x - blurSize, uv.y)) * 3.0/16.0;
    colour += texture2D(glowMap, vec2(uv.x, uv.y)) * 4.0/16.0;
    colour += texture2D(glowMap, vec2(uv.x + blurSize, uv.y)) * 3.0/16.0;
    colour += texture2D(glowMap, vec2(uv.x + 2.0*blurSize, uv.y)) * 2.0/16.0;
    colour += texture2D(glowMap, vec2(uv.x - 3.0*blurSize, uv.y)) * 1.0/16.0;

    gl_FragColor = colour;
}