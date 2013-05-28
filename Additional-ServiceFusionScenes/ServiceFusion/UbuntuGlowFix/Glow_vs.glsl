varying vec4 uv; 

void main()
{
    uv = gl_MultiTexCoord0;
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}