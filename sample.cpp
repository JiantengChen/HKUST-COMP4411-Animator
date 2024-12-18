
// The sample model.  You should build a file
// very similar to this for when you make your model.
#include "modelerview.h"
#include "modelerapp.h"
#include "modelerdraw.h"
#include <FL/gl.h>

#include "modelerglobals.h"
#include "particleSystem.h"
#include "modelerdraw.h"

#include "camera.h"
#include "bitmap.h"
#include <math.h>
#include <vector>

// To make a SampleModel, we inherit off of ModelerView
class SampleModel : public ModelerView
{
public:
    SampleModel(int x, int y, int w, int h, char *label)
        : ModelerView(x, y, w, h, label) {}

    virtual void draw();

private:
    Mat4f CameraMatrix;
};

// We need to make a creator function, mostly because of
// nasty API stuff that we'd rather stay away from.
ModelerView *createSampleModel(int x, int y, int w, int h, char *label)
{
    return new SampleModel(x, y, w, h, label);
}

// We are going to override (is that the right word?) the draw()
// method of ModelerView to draw out SampleModel
void SampleModel::draw()
{

    // This call takes care of a lot of the nasty projection
    // matrix stuff.  Unless you want to fudge directly with the
    // projection matrix, don't bother with this ...
    ModelerView::draw();

    Mat4f CameraMatrix = getModelViewMatrix();

    // start to draw the model.
    setAmbientColor(.1f, .1f, .1f);
    setDiffuseColor(0.4f, 0.4f, 0.4f);

    glPushMatrix();
    glTranslated(VAL(XPOS), VAL(YPOS), VAL(ZPOS));

    // part 1
    glPushMatrix();
    glTranslated(-2, 0, -1);
    glScaled(2, 3, 2);
    drawBox(1, 1, 1);
    glPopMatrix();

    // part 2
    glPushMatrix();
    glTranslated(0, 1.5, -1);
    glRotated(-90, 0.0, 0.0, 1.0);
    glScaled(1.5, 4.5, 2);

    glPushMatrix();
    drawBox(1, 1, 1);
    glPopMatrix();

    glPopMatrix();

    glPushMatrix();
    glTranslated(0, 1.5, -1);
    glScaled(4.5, 1.5, 2);
    drawBox(1, 1, 1);
    glPopMatrix();

    // LODS
    if (VAL(LOD) > 1)
    {

        // neck
        double angle0 = 20 + VAL(NECK_ANGLE);

        glPushMatrix();

        glTranslated(-1, 1.3, -1);
        glRotated(angle0, 0.0, 0.0, 1.0);

        glPushMatrix();
        glScaled(1.5, 10.0, 2);
        drawBox(1, 1, 1);
        glPopMatrix();

        // head
        if (VAL(LOD) > 2)
        {

            glTranslated(1.5, 8, 0);
            glRotated(90, 0, 0, 1);

            // rotate
            glTranslated(0, 0, 1);
            glRotated(VAL(HEAD_ANGLE_X), 1, 0, 0);
            glTranslated(0, 0, -1);
            glRotated(VAL(HEAD_ANGLE_Z), 0, 0, 1);

            glPushMatrix();
            glScaled(0.75, 3, 2);
            drawBox(1, 1, 1);
            glPopMatrix();

            glTranslated(0.75, 0, 0);

            glPushMatrix();
            glScaled(0.75, 2, 2);
            drawBox(1, 1, 1);
            glPopMatrix();

            glPushMatrix();
            glTranslated(0, 2, 0);
            glRotated(-90, 0, 0, 1);
            glScaled(0.4, 2.4, 0.4);
            drawBox(1, 1, 1);
            glPopMatrix();

            glPushMatrix();
            glTranslated(0, 2, 1.6);
            glRotated(-90, 0, 0, 1);
            glScaled(0.4, 2.4, 0.4);
            drawBox(1, 1, 1);
            glPopMatrix();
        }
        glPopMatrix();

        double frontLegSize = 0.6;
        double frontLegX = -2 + frontLegSize;
        double frontLegY = 0;
        double frontLegZ = -1;

        glPushMatrix();
        glTranslated(frontLegX, frontLegY + 0.5, frontLegZ);

        // rotate
        glRotated(VAL(RIGHT_FRONT_ANGLE1), 0, 0, 1);

        glPushMatrix();
        glRotated(-180, 0, 0, 1);
        glScaled(frontLegSize, 2 + 0.5, frontLegSize);
        drawBox(1, 1, 1);
        glPopMatrix();

        if (VAL(LOD) > 2)
        {

            glTranslated(0, -2 - 0.5 + 0.2, 0);
            glRotated(VAL(RIGHT_FRONT_ANGLE2), 0, 0, 1);

            glPushMatrix();
            glRotated(-180, 0, 0, 1);
            glScaled(frontLegSize, 1.5, frontLegSize);
            drawBox(1, 1, 1);
            drawBox(1, 1, 1);
            glPopMatrix();

            if (VAL(LOD) > 3)
            {

                glTranslated(0, -1.5 + 0.2, 0);
                glRotated(VAL(RIGHT_FRONT_ANGLE3), 0, 0, 1);

                glPushMatrix();
                glRotated(-180, 0, 0, 1);
                glScaled(1, 0.3, frontLegSize);
                drawBox(1, 1, 1);
                glPopMatrix();
            }
        }
        glPopMatrix();

        // front left leg
        frontLegZ = 1 - frontLegSize;

        // copied from above
        glPushMatrix();
        glTranslated(frontLegX, frontLegY + 0.5, frontLegZ);

        // rotate
        glRotated(VAL(LEFT_FRONT_ANGLE1), 0, 0, 1);

        glPushMatrix();
        glRotated(-180, 0, 0, 1);
        glScaled(frontLegSize, 2 + 0.5, frontLegSize);
        drawBox(1, 1, 1);
        glPopMatrix();

        glTranslated(0, -2 - 0.5 + 0.2, 0);
        glRotated(VAL(LEFT_FRONT_ANGLE2), 0, 0, 1);

        if (VAL(LOD) > 2)
        {
            glPushMatrix();
            glRotated(-180, 0, 0, 1);
            glScaled(frontLegSize, 1.5, frontLegSize);
            drawBox(1, 1, 1);
            glPopMatrix();

            if (VAL(LOD) > 3)
            {
                glTranslated(0, -1.5 + 0.2, 0);
                glRotated(VAL(LEFT_FRONT_ANGLE3), 0, 0, 1);

                glPushMatrix();
                glRotated(-180, 0, 0, 1);
                glScaled(1, 0.3, frontLegSize);
                drawBox(1, 1, 1);
                glPopMatrix();
            }
        }
        glPopMatrix();

        // back right leg

        double backLegSize = 0.6;
        double backLegX = 4.5;
        double backLegY = 1.7;
        double backLegZ = -1;

        double aux = 0.1;

        glPushMatrix();
        glTranslated(backLegX, backLegY + aux, backLegZ);

        //  rotate
        float alpha1, alpha2;
        const float l1 = 2.3, l2 = 3.0;
        glRotated(VAL(RIGHT_BACK_THETA), 1, 0, 0);
        glRotated(VAL(RIGHT_BACK_ANGLE1), 0, 0, 1);

        glPushMatrix();
        glRotated(-180, 0, 0, 1);
        glScaled(backLegSize, 2.5 + aux, backLegSize);
        drawBox(1, 1, 1);
        glPopMatrix();

        glTranslated(0, -2.5 + aux, 0);

        glRotated(VAL(RIGHT_BACK_ANGLE2), 0, 0, 1);

        if (VAL(LOD) > 2)
        {
            glPushMatrix();
            glRotated(-180, 0, 0, 1);
            glScaled(backLegSize, 3, backLegSize);
            drawBox(1, 1, 1);
            glPopMatrix();

            glTranslated(0, -3 + 3 * aux, 0);

            if (VAL(LOD) > 3)
            {

                // rotate
                glRotated(VAL(RIGHT_BACK_ANGLE3), 0, 0, 1);

                glPushMatrix();
                glRotated(-180, 0, 0, 1);
                glScaled(1, 0.3, backLegSize);
                drawBox(1, 1, 1);
                glPopMatrix();
            }
        }
        glPopMatrix();
        // left
        backLegZ = 1 - backLegSize;

        glPushMatrix();
        glTranslated(backLegX, backLegY + aux, backLegZ);
        // rotate
        glRotated(VAL(LEFT_BACK_ANGLE1), 0, 0, 1);

        glPushMatrix();
        glRotated(-180, 0, 0, 1);
        glScaled(backLegSize, 2.5 + aux, backLegSize);
        drawBox(1, 1, 1);
        glPopMatrix();

        glTranslated(0, -2.5 + aux, 0);
        // rotate
        glRotated(VAL(LEFT_BACK_ANGLE2), 0, 0, 1);

        if (VAL(LOD) > 2)
        {
            glPushMatrix();
            glRotated(-180, 0, 0, 1);
            glScaled(backLegSize, 3, backLegSize);
            drawBox(1, 1, 1);
            glPopMatrix();

            glTranslated(0, -3 + 3 * aux, 0);

            if (VAL(LOD) > 3)
            {
                // rotate
                glRotated(VAL(LEFT_BACK_ANGLE3), 0, 0, 1);

                glPushMatrix();
                glRotated(-180, 0, 0, 1);
                glScaled(1, 0.3, backLegSize);
                drawBox(1, 1, 1);
                glPopMatrix();
            }
        }
        glPopMatrix();

        // tail
        glPushMatrix();
        glTranslated(4.5, 2, -0.1);
        glRotated(VAL(TAIL_ANGLE_X), 1, 0, 0);
        glRotated(VAL(TAIL_ANGLE_Y), 0, 1, 0);

        glRotated(-150 + VAL(TAIL_ANGLE_Z), 0, 0, 1);

        // glPushMatrix();
        Mat4f CurrModelM = getModelViewMatrix();
        ParticleSystem *ps = ModelerApplication::Instance()->GetParticleSystem();
        // glPushAttrib(GL_ALL_ATTRIB_BITS);
        ps->spawnParticles(CameraMatrix, CurrModelM);
        // glPopAttrib();
        // glPopMatrix();

        glScaled(0.2, 2.5, 0.2);
        drawBox(1, 1, 1);

        glPopMatrix();
    }

    glPopMatrix();
    endDraw();
}

int main()
{
    // Initialize the controls
    // Constructor is ModelerControl(name, minimumvalue, maximumvalue,
    // stepsize, defaultvalue)
    ModelerControl controls[NUMCONTROLS];
    controls[XPOS] = ModelerControl("X Position", -5, 5, 0.1f, 0);
    controls[YPOS] = ModelerControl("Y Position", -5, 5, 0.1f, 0);
    controls[ZPOS] = ModelerControl("Z Position", -5, 5, 0.1f, 0);

    controls[LOD] = ModelerControl("Level of Detail", 1, 4, 1, 4);

    controls[LIGHT0_X] = ModelerControl("Light0 X Position", -5, 5, 0.1f, 2);
    controls[LIGHT0_Y] = ModelerControl("Light0 Y Position", -5, 5, 0.1f, 3);
    controls[LIGHT0_Z] = ModelerControl("Light0 Z Position", -5, 5, 0.1f, 4);

    controls[LIGHT1_X] = ModelerControl("Light1 X Position", -5, 5, 0.1f, -3);
    controls[LIGHT1_Y] = ModelerControl("Light1 Y Position", -5, 5, 0.1f, -3);
    controls[LIGHT1_Z] = ModelerControl("Light1 Z Position", -5, 5, 0.1f, -3);

    controls[LIGHT0_R] = ModelerControl("Light0 R Value", 0, 1, 0.01f, 0.3);
    controls[LIGHT0_G] = ModelerControl("Light0 G Value", 0, 1, 0.01f, 0.5);
    controls[LIGHT0_B] = ModelerControl("Light0 B Value", 0, 1, 0.01f, 0.7);

    controls[LIGHT1_R] = ModelerControl("Light1 R Value", 0, 1, 0.01f, 0.2);
    controls[LIGHT1_G] = ModelerControl("Light1 G Value", 0, 1, 0.01f, 0.7);
    controls[LIGHT1_B] = ModelerControl("Light1 B Value", 0, 1, 0.01f, 1);

    controls[NECK_ANGLE] = ModelerControl("Neck Angle", -45, 45, 0.1f, 1);
    controls[HEAD_ANGLE_X] = ModelerControl("Head Angle in X direction", -45, 45, 0.1f, 0);
    controls[HEAD_ANGLE_Z] = ModelerControl("Head Angle in Z direction", 0, 45, 0.1f, 0);
    controls[LEFT_FRONT_ANGLE1] = ModelerControl("Left front upper arm angle", -45, 45, 0.1f, 0);
    controls[LEFT_FRONT_ANGLE2] = ModelerControl("Left front middle arm angle", -45, 45, 0.1f, 0);
    controls[LEFT_FRONT_ANGLE3] = ModelerControl("Left front lower arm angle", -45, 45, 0.1f, 0);
    controls[RIGHT_FRONT_ANGLE1] = ModelerControl("Right front upper arm angle", -45, 45, 0.1f, 0);
    controls[RIGHT_FRONT_ANGLE2] = ModelerControl("Right front middle arm angle", -45, 45, 0.1f, 0);
    controls[RIGHT_FRONT_ANGLE3] = ModelerControl("Right front lower arm angle", -45, 45, 0.1f, 0);
    controls[LEFT_BACK_ANGLE1] = ModelerControl("Left back upper leg angle", -45, 45, 0.1f, 0);
    controls[LEFT_BACK_ANGLE2] = ModelerControl("Left back middle leg angle", -45, 45, 0.1f, 0);
    controls[LEFT_BACK_ANGLE3] = ModelerControl("Left back lower leg angle", -45, 45, 0.1f, 0);
    controls[RIGHT_BACK_ANGLE1] = ModelerControl("Right back upper leg angle", -45, 45, 0.1f, 0);
    controls[RIGHT_BACK_ANGLE2] = ModelerControl("Right back middle leg angle", -45, 45, 0.1f, 0);
    controls[RIGHT_BACK_ANGLE3] = ModelerControl("Right back lower leg angle", -45, 45, 0.1f, 0);

    controls[TAIL_ANGLE_X] = ModelerControl("Tail Angle X", -45, 45, 0.1f, 0);
    controls[TAIL_ANGLE_Y] = ModelerControl("Tail Angle Y", -45, 45, 0.1f, 0);
    controls[TAIL_ANGLE_Z] = ModelerControl("Tail Angle Z", -15, 135, 0.1f, 0);
    controls[RIGHT_BACK_THETA] = ModelerControl("Right back upper leg theta", -15, 15, 1, 0);

    // particle system
    ParticleSystem *ps = new ParticleSystem();
    ModelerApplication::Instance()->SetParticleSystem(ps);

    ModelerApplication::Instance()->Init(&createSampleModel, controls, NUMCONTROLS);
    return ModelerApplication::Instance()->Run();
}
