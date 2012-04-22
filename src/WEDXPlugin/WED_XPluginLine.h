/*
 * Copyright (c) 2012, mroe.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */
#ifndef WED_XPLUGINLINE_H
#define WED_XPLUGINLINE_H

#include "WED_XPluginEntity.h"
#include "WED_XPluginBezierChain.h"


class WED_XPluginMgr;

class WED_XPluginLine : public WED_XPluginEntity ,public WED_XPluginBezierChain
{
    public:

        WED_XPluginLine(WED_XPluginMgr * inRef,const vector<string>& inArgs);
        WED_XPluginLine(WED_XPluginMgr * inRef);
        virtual ~WED_XPluginLine();

        void     Draw(bool isLit);
        void     Update(const vector<string>& inArgs);
        void     SceneryShift(){;}
        int      SetRessource(const string& inPath);

    protected:
    private:

        int       mIsClosed;
};

#endif // WED_XPLUGINLINE_H
