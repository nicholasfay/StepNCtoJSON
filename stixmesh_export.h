/* $RCSfile: stixmesh_export.h,v $
 * $Revision: 1.12 $ $Date: 2015/04/30 18:48:47 $
 * Auth: Jochen Fritz (jfritz@steptools.com)
 * 
 * 	Copyright (c) 1991-2015 by STEP Tools Inc.
 * 	All Rights Reserved
 * 
 * 	This software is furnished under a license and may be used and
 * 	copied only in accordance with the terms of such license and with
 * 	the inclusion of the above copyright notice.  This software and
 * 	accompanying written materials or any other copies thereof may
 * 	not be provided or otherwise made available to any other person.
 * 	No title to or ownership of the software is hereby transferred.
 * 
 * 		----------------------------------------
 */

#ifndef STIXMESH_EXPORT_H
#define STIXMESH_EXPORT_H

#ifndef   STIXMESH_DEFS_H
#include "stixmesh_defs.h"
#endif

#include "stixmesh_simple.h"

class RoseXMLWriter;
class stp_representation;
class stp_shape_representation;

class StixMeshStpBuilder;

class STIXMESH_DLLSPEC StixMeshXMLExport {

    StixMeshStpAsyncMaker async;
    
    unsigned count;
    unsigned size;
    
    RoseXMLWriter * main_xml;
    const char * out_dir;
    StixMeshOptions * opts;
    
public:
    
    StixMeshXMLExport(
	RoseXMLWriter * x,
	const char * dir,
	StixMeshOptions * o=0,
	unsigned sz=0
	)
    {
	main_xml = x;
	out_dir = dir;	
	size=sz;
	opts = o;
	count=0;
    }

    StixMeshXMLExport(
	RoseXMLWriter * x,
	const char * dir,
	unsigned sz
	)
    {
	main_xml = x;
	out_dir = dir;	
	size=sz;
	opts = 0;
	count=0;
    }

    virtual ~StixMeshXMLExport() {}
    
    StixMeshStpAsyncMaker * getAsync() {return &async;}

    RoseXMLWriter * getMainXML() {return main_xml;}

    const char * getOutputDir() {return out_dir;}
    
    /*
     * Append a shell to the task queue
     */
    void appendShell(stp_representation * rep, stp_representation_item * ri);

    /*
     * Flush the completed shells.  Returns true if all the shells have been
     * written.  (Some may still be running in the BG.)
     * If wait is true, do not return until all the pending tasks have been
     * completed.
     */
    int flushCompletedShells(int wait);

    virtual void reportProgress(stp_representation_item * it);

    void appendRef (
	RoseXMLWriter * xml,	
	const char * name, 
	RoseObject * val
	);
    
    void appendVal (
	RoseXMLWriter * xml,
	const char * name,
	double val);

    void dumpShapeAsm(
	RoseXMLWriter * xml,
	stp_representation * sr,
	rose_uint_vector * roots
    );

    void dumpShapeAsm(
	stp_representation * sr,
	rose_uint_vector * roots)
    {
	dumpShapeAsm(main_xml, sr, roots);
    }
    
    /* append a shell="ids" attribute for the shells in the given shape rep*/
    void appendShellRefs(
	RoseXMLWriter * xml,
	stp_representation * sr
	);

    /* Write out the shells in the shape rep */
    void appendShells (stp_representation * sr);
    
};


STIXMESH_EXTERN unsigned stixmesh_count_shapes_asm(
    stp_representation * sr,
    rose_uint_vector * roots
    );


STIXMESH_EXTERN int stixmesh_has_shell(
    stp_shape_representation * sr
    );




#endif
