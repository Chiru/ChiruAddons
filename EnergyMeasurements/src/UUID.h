
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#ifndef UUID_H
#define UUID_H

// Class definition

class UUID
{
public:
    static UUID * Instance();
    ~UUID();

    unsigned int getUUID(void);

private:
    // singleton
    UUID() { UniqueID = 0; }                    // Private constructor
    UUID(UUID const &) { }
    static UUID *p_Instance;                    // Single instance placeholder
    unsigned int UniqueID;
};

#endif // UUID_H
