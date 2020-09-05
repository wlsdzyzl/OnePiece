#version 130

    in vec3 position;
    uniform mat4 MVP;
    uniform int colorType;

    uniform float materialShininess;
    uniform vec4 materialAmbient;
    uniform vec4 materialDiffuse;
    uniform vec4 materialSpecular;
    uniform vec4 lightAmbient;
    uniform vec4 lightDiffuse;
    uniform vec4 lightSpecular;


out vec4 vColor;

void main()
{
        vColor = vec4(0.0, 0.0, 0.0, 1.0);
	    gl_Position = MVP * vec4(position.xyz, 1.0);
        
}