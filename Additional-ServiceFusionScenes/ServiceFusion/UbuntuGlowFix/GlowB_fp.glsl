uniform sampler2D TEX1;
uniform sampler2D TEX2;

uniform vec4 invTexSize;
uniform float GlowBrightness;
uniform float GlowBlurSize;

varying vec4 uv;

void main()
{
    vec4 colour = vec4(0.0, 0.0, 0.0, 0.0);
    float blurSize = GlowBlurSize * 0.0003;

    //X-blur.
    colour += texture2D(TEX2, vec2(uv.x, uv.y - 3.0*blurSize)) * 1.0/16.0;
    colour += texture2D(TEX2, vec2(uv.x, uv.y - 2.0*blurSize)) * 2.0/16.0;
    colour += texture2D(TEX2, vec2(uv.x, uv.y - blurSize)) * 3.0/16.0;
    colour += texture2D(TEX2, vec2(uv.x, uv.y)) * 4.0/16.0;
    colour += texture2D(TEX2, vec2(uv.x, uv.y - blurSize)) * 3.0/16.0;
    colour += texture2D(TEX2, vec2(uv.x, uv.y + 2.0*blurSize)) * 2.0/16.0;
    colour += texture2D(TEX2, vec2(uv.x, uv.y + 3.0*blurSize)) * 1.0/16.0;

    gl_FragColor = texture2D(TEX1, vec2(uv.x, uv.y)) + colour * GlowBrightness;
}