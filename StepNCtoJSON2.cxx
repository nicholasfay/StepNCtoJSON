/* $RCSfile: dump_project.cxx,v $
 * $Revision: 1.18 $ $Date: 2015/07/16 16:22:59 $
 * Auth: Nicholas Fay (fayn@rpi.edu)
 * 
 * 	Copyright (c) 1991-2014 by STEP Tools Inc.
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
 * 
 *  Convert a STEP-NC file to JSON for WebGL comsumption.
 */


#include <math.h>
#include <stp_schema.h>
#include <stix.h>
#include <stixmesh_error.h>
#include <iostream>
#include <cstdlib>

#include "stixmesh_writer.h"
#include "stp2webgl.h"
#include "json.h"

#include <Compensation_workingstep.h>
#include <Fixture_usage.h>
#include <Frame_definition_workingstep.h>
#include <Machining_operation_IF.h>
#include <Machining_tool_IF.h>
#include <Machining_workingstep.h>
#include <Milling_machine_cutting_tool_IF.h>
#include <Old_touch_probe.h>
#include <Project.h>
#include <Selective.h>
#include <Setup.h>
#include <Tool_usage.h>
#include <Touch_probe.h>
#include <Workpiece.h>
#include <Workpiece_probing_IF.h>
#include <Workplan.h>


/* AP238 ARM classes */
#include <Cutter_contact_trajectory.h>
#include <Cutter_location_trajectory.h>
#include <Milling_technology.h>

#include <stixmesh_nurbs.h>
#include <stixmesh_present.h>
#include "stixmesh_export.h"



#ifdef _WIN32
#include <direct.h>
#else
#include <unistd.h>	/* for mkdir () */
#endif

#include <sys/types.h> 	/* for mkdir */
#include <sys/stat.h>
#include <errno.h>


#ifdef _WIN32
#define MKDIR_PROT_PARM
#define DIR_ALREADY_EXISTS (errno == EEXIST)
#else
/* maybe this should use the umask instead */
#define MKDIR_PROT_PARM   ,0775
#define DIR_ALREADY_EXISTS (errno == EEXIST)
#define CLASS_PATH_BAK_EXT	".bak"
#endif

using namespace Json;

stp_product_definition * stmfg_get_tool_product(
    stp_machining_operation * op
    );

stp_product_definition * stmfg_get_tool_product(
    stp_machining_tool * mt
    );

void dumpShapeAsm(
    Value& json,
    Value& shape,
    Value & shellArr,
    stp_representation * sr,
    rose_uint_vector * roots,
    const char * fn,
    StixMeshStpAsyncMaker& async,
    StixMeshOptions * opts);


/************************************************/
static void add_ref_id (RoseObject * obj, char * buffer)
{
    sprintf (buffer, "id%d", obj->entity_id());
}


/************************************************/

stp_shape_representation * get_workpiece_geometry(stp_product_definition * pd)
{
    Workpiece * wp = Workpiece::find(pd);
    if (!wp) {
	return 0;
    }

    return wp->get_its_geometry();
}


static stp_machining_tool * get_tool(Machining_operation_IF * op)
{
    if (!op)
	return 0;

    return op->get_its_tool();
    
}

static Milling_machine_cutting_tool_IF * get_arm_tool(
    Machining_operation_IF * op)
{
    stp_machining_tool * tool = get_tool(op);

    Machining_tool_IF * ret = Machining_tool_IF::find(tool);
    if (!tool) {
	printf ("Could not get tool\n");
	exit (2);
    }

    return ARM_CAST(Milling_machine_cutting_tool_IF, ret);
    
}

static double get_tool_length(Machining_operation_IF * op)
{
    Milling_machine_cutting_tool_IF * udmt = get_arm_tool(op);
    if (!udmt)
	return 0;
    
    stp_measure_representation_item * mri
	= udmt->get_overall_assembly_length();

    /* FIXME: convert units */
    return stix_get_measure_value(mri, stixunit_as_is);

}



static double get_tool_length(Workpiece_probing_IF * op)
{
    Touch_probe_IF * tool = Touch_probe_IF::find(op->get_its_probe());
    
    if (!tool)
	return 0.;

    stp_measure_representation_item * mri
	= tool->get_overall_assembly_length();

    /* FIXME: convert units */
    return stix_get_measure_value(mri, stixunit_as_is);
    
}



static stp_product_definition * get_tool_product(
    double * len,
    stp_machining_operation * op
    )
{
    {
	Machining_operation_IF * armop = Machining_operation_IF::find(op);
	stp_machining_tool * mt =  armop? armop->get_its_tool(): 0;

//	stp_machining_tool * mt = stmfg_get_tool(op);
	if (mt) {
	    *len = get_tool_length(armop);
	    return stmfg_get_tool_product(mt);
	}
    }

    // if no tool, then we might be dealing with the special case of a
    // touch probe, which the IS did not make an actual tool.  We use
    // a specially marked workpiece for that.  This will become a real
    // tool in the TC/E2.
    //
    Workpiece_probing_IF * wp = Workpiece_probing_IF::find(op);
    if (!wp) return 0;

    *len = get_tool_length(wp);
    
    // look first for the TC/E2 probe-as-a-conventional-tool usage.
    if (wp->get_its_probe()) {
	return stmfg_get_tool_product(wp->get_its_probe());
    }

    // next, look for orig IS probe as a stub entity.
    Old_touch_probe * old_probe = Old_touch_probe::find(
	wp->get_its_old_probe());
    if (!old_probe) return 0;

    const char * probe_id = old_probe->get_its_id();
    if (!probe_id) return 0;

    // Find first tool usage in the file with a matching name.
    //
    ARMCursor cur;
    cur.traverse(op->design());
    cur.domain(Tool_usage::type());
    Tool_usage * tu;

    while ((tu=ARM_CAST(Tool_usage, cur.next())) != 0) 
    {
	const char * tu_id = tu->get_its_id();
	if (tu_id && !strcmp(tu_id, probe_id)) 
	{
	    // Why is this searching for a workpiece?  We should just
	    // be able to return the its_product.  The only thing this
	    // does is check to make sure that it has been recognized
	    // as a workpiece.
	    Workpiece * wp = Workpiece::find(tu->get_its_product());
	    if (wp) return wp->getRoot();
	}
    }
    return 0;
}


static double get_mri_value(stp_measure_with_unit * mri)
{
    if (!mri)
	return ROSE_NULL_REAL;
    
    stp_measure_value * val = mri->value_component();
    if (!val)
	return ROSE_NULL_REAL;
    return val->getDouble(val->getAttribute());
}


static double get_speed_per_sec(stp_measure_with_unit * mu, StixUnit len_unit)
{
    StixUnit speed_un = stix_get_unit_type(mu->unit_component());

    /* Component units of speed unit */
    StixUnit speed_len;
    StixUnit speed_time;

    switch(speed_un) {
    case stixunit_mmps:
	speed_len = stixunit_mm;
	speed_time = stixunit_sec;
	break;
    case stixunit_mmpm:
	speed_len = stixunit_mm;
	speed_time = stixunit_min;
	break;
    case stixunit_cmps:
	speed_len = stixunit_cm;
	speed_time = stixunit_sec;
	break;
    case stixunit_mps:
	speed_len = stixunit_m;
	speed_time = stixunit_sec;
	break;
    case stixunit_ips:
	speed_len = stixunit_in;
	speed_time = stixunit_sec;
	break;
    case stixunit_ipm:
	speed_len = stixunit_in;
	speed_time = stixunit_min;
	break;
    case stixunit_fps:
	speed_len = stixunit_ft;
	speed_time = stixunit_sec;
	break;
    case stixunit_fpm:
	speed_len = stixunit_ft;
	speed_time = stixunit_min;
	break;

    default:
	printf ("unknown speed: %d\n", speed_un);
	exit (2);
    }

    double factor = stix_get_unit_conversion_factor(speed_len, len_unit) /
	stix_get_unit_conversion_factor (speed_time, stixunit_sec);

    return get_mri_value(mu) * factor;
}



double get_tech_feedrate(stp_machining_technology * tech, StixUnit len_unit)
{
    Milling_technology * arm = Milling_technology::find(tech);

    if (arm->isset_feedrate()) {

	stp_measure_representation_item * mri = arm->get_feedrate();
	if (!mri)
	    return ROSE_NULL_REAL;

	return get_speed_per_sec(mri, len_unit);

    }

    return 0.;
}

/* The state for a toolpath */
struct TPState {

    /* The tool paths */
    StixMeshNurbs bcurve;
    StixMeshNurbs axis;

    double u, d, t, s;  /* parameter, distance, time, feedrate(speed) */

    TPState() {
	t = d = 0.;
    }
};


static void append_vec3(Value& json, double v[3])
{
    for (int i=0; i<3; i++) {
	json.append(v[i]);
    }
}


static void write_tp_point(
    Value& tp_point,
    TPState * st)
{
    double u = st->u;
    double xyz[3];
    st->bcurve.eval(xyz, u);
    
    tp_point["u"] = u;
    
    if (st->d != 0.) {
	tp_point["d"] = st->d;
    }
    
    if (st->t != 0.) {
	tp_point["t"] = st->t;
    }

    tp_point["l"].append(xyz[0]);
    tp_point["l"].append(xyz[1]);
    tp_point["l"].append(xyz[2]);
   
    if (!st->axis.isEmpty()) {
	double ijk[3];
	st->axis.eval(ijk, u);
	for (int i = 0; i < 3; i++){
	    tp_point["l"].append(ijk[i]);
	}
    }
}


static void normalize(double v[3])
{
    double len = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (fabs(len) < 1e-20) {
	printf ("Zero length\n");
	exit (2);
    }

    v[0] /= len;
    v[1] /= len;
    v[2] /= len;
}



static void insert_axis_points(
    rose_real_vector * pts,
    StixMeshNurbs *axis_nurbs,
    double u1, double u2)
{
    double ijk1[3];
    axis_nurbs->eval(ijk1, u1);
    double ijk2[3];
    axis_nurbs->eval(ijk2, u2);

    normalize(ijk1);
    normalize(ijk2);
    
    double dot =
	ijk1[0]*ijk2[0] +
	ijk1[1]*ijk2[1] +
	ijk1[2]*ijk2[2];

    if (dot >= .9) {
	return;
    }
    
    double m = (u1+u2) / 2.;
    insert_axis_points(pts, axis_nurbs, u1, m);
    pts->append(m);
    insert_axis_points(pts, axis_nurbs, m, u2);
}

static void append_tp_point(
    Value& tp_point,
    TPState * st,
    double u)
{
    double old_u = st->u;
    double len = st->bcurve.arcLength(old_u, u);

    st->d += len;
    if (st->s != 0)
	st->t += len/st->s;
    
    st->u = u;
    write_tp_point(tp_point, st);
}



static double get_rot_value(stp_measure_representation_item * mri)
{
    StixUnit un = stix_get_unit_type(mri->unit_component());

    switch (un) {
    case stixunit_hertz:
	return get_mri_value(mri);

    case stixunit_rpm:
	return get_mri_value(mri) / 60.;

    default:
	printf ("get_rot_value: unexpected unit=%d\n", un);
	exit (2);
    }
    
}


static void dump_tech(
    Value& techJ,
    stp_machining_technology * tech,
    StixUnit len_unit)
{
    if (tech->wasVisited())
	return;

    tech->visit();
    
    {
	Milling_technology * arm = Milling_technology::find(tech);
	if (arm) {
	    char buff[20];
	    add_ref_id(tech, buff);
	    techJ["id"] = buff;

	    if (arm->isset_feedrate()) {

		techJ["feedrate"] = get_tech_feedrate(tech, len_unit);
	    }

	    if (arm->isset_spindle()) {
		techJ["spindle"] = get_rot_value(arm->get_spindle());
	    }
	    return;
	}
    }

    printf ("Unimplemented technology case: #%d\n", tech->entity_id());
    exit (2);

}


static FILE * open_dir_file(const char * dir, const char * fname)
{
    RoseStringObject path = dir;
    path.cat("/");
    path.cat(fname);
 
    return fopen(path, "w");
}

static void append_bbox(
    Value& value,
    StixMeshBoundingBox * bbox)
{
    value.append(bbox->minx);
    value.append(bbox->miny);
    value.append(bbox->minz);
    value.append(bbox->maxx);
    value.append(bbox->maxy);
    value.append(bbox->maxz);
}

static void dump_toolpath(
    Value& json,
    Value& tooldump,
    Trajectory_IF * tp,
    stp_bounded_curve * bcurve,
    stp_representation * bcurve_rep,
    stp_bounded_curve * axis,
    stp_representation * axis_rep,
    double tol,
    const char * fn)
{
    stp_machining_technology * tech = tp->get_its_technology();

    StixUnit lun = stix_get_context_unit(bcurve_rep, stixvalue_length);

    double feed = get_tech_feedrate(tech, lun);

    const char * out_dir = fn;

    
    RoseBoolean rapid_speed = tp->isset_rapid_speed();
    RoseBoolean ratio_speed = tp->isset_ratio_speed();
    RoseBoolean curve_speed = tp->isset_curve_speed();

    double speed = feed;

    if (rapid_speed) {
	speed = 0.;
    }
    else if (ratio_speed) {
	printf ("Cannot handle ratio speed\n");
	exit (2);
    }
    
    else if (curve_speed) {
	printf ("WARNING: cannot handle curve speed: \n");
    }   
    
    TPState stat;
    
    stat.s = speed;
    
    if (!stixmesh_create_bounded_curve(&stat.bcurve, bcurve, bcurve_rep)) {
	printf ("dump_toolpath: create_bounded_curve failed\n");
	exit (2);
    }
    
    if (axis) {
	if (!stixmesh_create_bounded_curve(&stat.axis, axis, axis_rep)) {
	    printf ("dump_toolpath: create_bounded_curve failed 2\n");
	    exit (2);
	}
    }
    
    StixMeshBoundingBox bbox;
    stat.bcurve.getConvexHull(&bbox);
    
    if (tol == ROSE_NULL_REAL) {
	tol = bbox.diagonal() / 100;
    }

    rose_real_vector u_vals;
    if (tol > 0.) {
	stat.bcurve.extractTolerancedPoints(&u_vals, tol, 1);
    }
    
    if (out_dir) {

	char id[100];
	sprintf (id, "id%lu", tp->getRootObject()->entity_id());

	RoseStringObject fname = "toolpath_";
	fname.cat(id);
	fname.cat (".JSON");

	tooldump["id"] = id;

	char sz_buff[10];
	sprintf (sz_buff, "%u", u_vals.size());
	tooldump["size"] = sz_buff;

	append_bbox(tooldump["bbox"], &bbox);

	char id2[64];
	sprintf(id2, "%s", fname);
	tooldump["href"] = id2;

	FILE * fd = open_dir_file(fn, fname);
	
	Value exportVal;

	char buff[20];
	add_ref_id(tp->getRootObject(), buff);
	exportVal["id"] = buff;


	/* We may have a degenerate toolpath.  If so, output nothing */

	if (u_vals.size() > 0.) {
	    stat.bcurve.extractTolerancedPoints(&u_vals, tol, 1);

	    double prev_u = u_vals[0];

	    stat.u = u_vals[0];
	    Value testVal;
	    write_tp_point(testVal, &stat);
	    exportVal["p"].append(testVal);

	    for (unsigned i = 1; i<u_vals.size(); i++) {

		double u = u_vals[i];
		Value point;
		rose_real_vector pushed_points_axis;

		if (axis) {
		    insert_axis_points(&pushed_points_axis, &stat.axis,
			prev_u, u);
		}

		for (unsigned j = 0; j<pushed_points_axis.size(); j++) {
		    Value point2;
		    append_tp_point(point2, &stat, pushed_points_axis[j]);
		    exportVal["p"].append(point2);
		}

		append_tp_point(point, &stat, u);
		exportVal["p"].append(point);

		prev_u = u;
	    }
	}

	//FastWriter writer;
	StyledWriter writer;
	std::string output = writer.write(exportVal);
	const char * c = output.c_str();

	if (fd != NULL){
	    fputs(c, fd);
	    fclose(fd);
	}

	char buff2[30];
	sprintf(buff2, "%f", stat.d);
	tooldump["length"] = buff2;
    } 

    char buff[20];
    add_ref_id(tp->getRootObject(), buff);
    tooldump["id"] = buff;


    /* We may have a degenerate toolpath.  If so, output nothing */

    if (u_vals.size() > 0.) {
	stat.bcurve.extractTolerancedPoints(&u_vals, tol, 1);

	double prev_u = u_vals[0] ;

	stat.u = u_vals[0];
	Value testVal;
	write_tp_point(testVal, &stat);
	tooldump["p"].append(testVal);

	for (unsigned i=1; i<u_vals.size(); i++) {

	    double u = u_vals[i];
	    Value point;
	    rose_real_vector pushed_points_axis;
	    
	    if (axis) {
		insert_axis_points(&pushed_points_axis, &stat.axis,
				   prev_u, u);
	    }
	    
	    for (unsigned j=0; j<pushed_points_axis.size(); j++) {
		Value point2;
		append_tp_point(point2, &stat, pushed_points_axis[j]);
		tooldump["p"].append(point2);
	    }
	    
	    append_tp_point(point, &stat, u);
	    tooldump["p"].append(point);
	    
	    prev_u = u;
	}
    }

    Value techJ;
    dump_tech(techJ, tech, lun);
    if (techJ != Value::null)
	json["milling_technology"].append(techJ);
}


static void dump_toolpath(
    Value& json,
    Value& tooldump,
    Cutter_location_trajectory * tp,
    double tol,
    const char * fn)

{
    stp_bounded_curve * bcurve = tp->get_basiccurve();
    stp_representation * bcurve_rep = tp->get_basiccurve_rep();    
    stp_bounded_curve * axis = tp->get_its_toolaxis();
    stp_representation * axis_rep = 0;
    
    if (axis)
	axis_rep = tp->get_its_toolaxis_rep();

    dump_toolpath(json, tooldump, tp, bcurve, bcurve_rep, axis, axis_rep, tol, fn);
}


static void dump_toolpath(
    Value& json,
    Value& tooldump,
    Cutter_contact_trajectory * tp,
    double tol,
    const char * fn)
{
    stp_bounded_curve * bcurve = tp->get_basiccurve();
    stp_representation * bcurve_rep = tp->get_basiccurve_rep();    
    stp_bounded_curve * axis = tp->get_its_toolaxis();
    stp_representation * axis_rep = 0;
    
    if (axis)
	axis_rep = tp->get_its_toolaxis_rep();

    dump_toolpath(json, tooldump, tp, bcurve, bcurve_rep, axis, axis_rep, tol, fn);
    printf ("WARNING: cutter_contact_trajectory: surface normal not yet implemented\n");
    
}


void stgl_xml_dump_toolpath(
    Value& json,
    Value& tooldump,
    stp_machining_toolpath * tp,
    const char * fn,
    double tol = ROSE_NULL_REAL)
{
    if (tp->wasVisited())
	return;
    
    tp->visit();
    
    {
	Cutter_location_trajectory * arm_tp
	    = Cutter_location_trajectory::find(tp);

	if (arm_tp) {
	    dump_toolpath(json, tooldump, arm_tp, tol,fn);
	    return;
	}
    }

    {
	Cutter_contact_trajectory * arm_tp
	    = Cutter_contact_trajectory::find(tp);

	if (arm_tp) {
	    dump_toolpath(json, tooldump, arm_tp, tol,fn);
	    return;
	}
    }
    
    printf ("stgl_xml_dump_toolpath: Unimplemented toolpath case: #%d\n",
	    tp->entity_id());
    exit(2);
}



static void dump_operation (
    Value& json,
    Value & shellArr,
    Value& operation,    
    stp_machining_operation * op,
    const char * fn,
    StixMeshStpAsyncMaker& async,
    StixMeshOptions * opts)
{
    if (!op) {
	printf ("No AIM operation\n");
	exit (2);
    }

    if (op->wasVisited())
	return;

    op->visit();
    
    Operation_IF * arm_op = Operation_IF::find(op);
    if (!arm_op) {
	printf ("No operation from #%d (%s)\n",
		op->entity_id(), op->domain()->name());
	exit (2);
    }

    double tool_len;
    stp_shape_representation * tool_geom = get_workpiece_geometry(
	get_tool_product(&tool_len, op));
    
    char buff[20];
    add_ref_id(op, buff);
    operation["id"] = buff;

    unsigned sz = arm_op->size_its_toolpath();
    
    if (sz > 0) {
	Value toolpath;
	for (unsigned i=0; i<sz; i++) {
	    stp_machining_toolpath * tp
		= arm_op->get_its_toolpath(i)->getValue();
	    char buff[20];
	    add_ref_id(tp, buff);
	    toolpath.append(buff);
	}
	operation["toolpath"] = toolpath;
    }

    if (tool_geom) {
	char buff[20];
	add_ref_id(tool_geom, buff);
	operation["tool"] = buff;

	char buff2[20];
	sprintf (buff2, "%g", tool_len);
	operation["tool_length"] = buff2;
    }

    for (unsigned i=0; i<sz; i++) {
	stp_machining_toolpath * tp
	    = arm_op->get_its_toolpath(i)->getValue();
	Value toolpath;
	stgl_xml_dump_toolpath(json, toolpath,tp,fn);
	json["toolpath"].append(toolpath);
    }

    Value shapeJ;
    dumpShapeAsm(json, shapeJ, shellArr, tool_geom, 0,fn, async, opts);
    if (shapeJ != Value::null)
	json["shape"].append(shapeJ);
}



static void add_xform(Value& input, const char * att,
		      stp_axis2_placement_3d * place)
{
    if (place == 0)
	return;

    StixMtrx xform(place);
    
    unsigned i,j;
    Value xformarr;
    for (i=0; i<4; i++)
	for (j=0; j<4; j++) {
	    char buff[20];
	    sprintf(buff, "%s%g", (i == 0 && j == 0) ? "" : " ", xform.get(j, i));
	    xformarr.append(buff);
	}
	input[att] = xformarr;
}

/* FIXME: consider using a more compact form for the matrix.
 * ex: it we have an identity transform, onit the transform;
 * if we have just a translation, include just the xyz of the translation.
 */
void dump_placement (
    Value& placeJ, 
    stp_axis2_placement_3d * placement)
{
    if (!placement || placement->wasVisited())
	return;

    placement->visit();
    
    char buff[20];
    add_ref_id(placement, buff);
    placeJ["id"] = buff;

    StixMtrx xform(placement);
    
    unsigned i,j;
    Value xformarr;
    for (i = 0; i < 4; i++){
	for (j = 0; j < 4; j++) {
	    char buff[20];
	    sprintf(buff, "%s%g", (i == 0 && j == 0) ? "" : " ", xform.get(j, i));
	    xformarr.append(buff);
	}
    }

    placeJ["xform"] = xformarr;
}

stp_shape_representation * get_shape(stp_product_definition * pd)
{
    if (pd == 0)
	return 0;
    
    Workpiece * wp = Workpiece::find(pd);
    if (wp == 0)
	return 0;

    return wp->get_its_geometry();
}

void appendShellRefs(
    Value& shape,
    stp_representation * sr)
{
    SetOfstp_representation_item * items = sr->items();
    unsigned sz = items->size();

    unsigned count = 0;
    Value ids;

    for (unsigned i = 0; i<sz; i++) {
	stp_representation_item * it = items->get(i);

	if (!StixMeshStpBuilder::isShell(sr, it))
	    continue;

	char buff[20];
	add_ref_id(it,buff);
	ids.append(buff);
    }
    if (ids != Value::null)
	shape["shell"] = ids;
}

static void append_annotation_refs(
    Value& shape,
    stp_representation * sr)
{
    /* FIXME - this generates both constructive geometry and annotations.
    * theses should be split out, but are are not (currently) doing so since
    * that would require updating the webgl javascript.
    */

    if (!sr->isa(ROSE_DOMAIN(stp_shape_representation)))
	return;

    StixMeshRepresentationVec * models = stixmesh_get_draughting_models(
	ROSE_CAST(stp_shape_representation, sr));

    StixMeshConstructiveGeomVec * cgeom = stixmesh_get_constructive_geometry(sr);

    if (!models && !cgeom)
	return;

    if (models) {
	unsigned sz = models->size();
	if (sz > 0) {
	    Value ids;
	    bool b = false;
	    if (sz > 1)
		b = true;
	    for (unsigned i = 0; i<sz; i++) {
		stp_representation * model = models->get(i);
		char buff[20];
		add_ref_id(model, buff);
		if (b)
		    ids.append(buff);
		else
		    ids = buff;
	    }
	    shape["annotation"] = ids;
	}
    }

    if (cgeom) {
	unsigned sz = cgeom->size();
	if (sz > 0) {
	    Value ids;
	    bool b = false;
	    if (sz > 1)
		b = true;
	    for (unsigned i = 0; i<sz; i++) {
		stp_constructive_geometry_representation  * rep = cgeom->get(i);
		char buff[20];
		add_ref_id(rep,buff);
		ids.append(buff);
	    }
	    shape["annotation"] = ids;
	}
    }
}

static void append_asm_child(
    Value& shape,
    RoseObject * rel
    )
{
    StixMgrAsmRelation * rm = StixMgrAsmRelation::find(rel);
    if (!rm) {
	stixmesh_ec()->error("Could not find RM\n");
	return;
    }

    stp_representation * child = rm->child;

    char buff[20];
    add_ref_id(child, buff);
    shape["ref"] = buff;


    StixMtrx xform = stix_get_transform(rm);

    unsigned i, j;
    Value xformarr;
    for (i = 0; i < 4; i++){
	for (j = 0; j < 4; j++) {
	    char buff[20];
	    sprintf(buff, "%s%g", (i == 0 && j == 0) ? "" : " ", xform.get(j, i));
	    xformarr.append(buff);
	}
    }
    shape["xform"] = xformarr;
}

static void dump_shape_child(
    Value& json,
    Value& shape,
    Value& shellArr,
    RoseObject * rel,
    rose_uint_vector * roots,
    const char * fn,
    StixMeshStpAsyncMaker& async,
    StixMeshOptions * opts)
{
    StixMgrAsmRelation * rm = StixMgrAsmRelation::find(rel);
    if (!rm) {
	stixmesh_ec()->error("Could not find RM\n");
	return;
    }

    stp_representation * child = rm->child;

    dumpShapeAsm(json, shape, shellArr, child, roots,fn,async,opts);
}

void appendShell(
    stp_representation * rep,
    stp_representation_item * ri,
    StixMeshStpAsyncMaker& async,
    StixMeshOptions* opts)
{
    if (!StixMeshStpBuilder::canMake(rep, ri))
	return;
    async.startMesh(rep, ri, opts);
}

void appendShells(
    stp_representation * sr,
    StixMeshStpAsyncMaker& async,
    StixMeshOptions* opts
)
{
    SetOfstp_representation_item * items = sr->items();
    unsigned sz = items->size();

    for (unsigned i = 0; i<sz; i++) {
	appendShell(sr, items->get(i), async, opts);
    }

}

static void write_poly_point(Value& point, ListOfDouble * vals)
{
    point.append(vals->get(0));
    point.append(vals->get(1));
    point.append(vals->get(2));
}

static void append_step_curve(Value& polyline, stp_curve * c)
{
    /* May also want to handle composite curves here in this routine, since
    * a composite curve may contain via points, and thus run into trouble.*/
    unsigned i, sz;

    if (c->isa(ROSE_DOMAIN(stp_polyline))) {

	stp_polyline * poly = ROSE_CAST(stp_polyline, c);
	ListOfstp_cartesian_point * pts = poly->points();

	if (!pts || pts->size() < 2)
	    return;

	for (i = 0, sz = pts->size(); i<sz; i++) {
	    stp_cartesian_point * pt = pts->get(i);
	    ListOfDouble * vals = pt->coordinates();
	    Value point;
	    write_poly_point(point["p"], vals);
	    polyline.append(point);
	}
    }

}

static void export_annotation(Value& poly,
    stp_geometric_set * gset)
{
    SetOfstp_geometric_set_select * elems = gset->elements();
    if (!elems) return;

    unsigned i, sz;
    for (i = 0, sz = elems->size(); i<sz; i++) {
	stp_geometric_set_select * sel = elems->get(i);
	if (sel->is_curve()) {
	    Value poly2;
	    append_step_curve(poly2, sel->_curve());
	    poly["polyline"].append(poly2);
	}
	else if (sel->is_point()) {
	}
	else if (sel->is_surface()) {
	}

    }
}

static void export_annotation(
    Value& poly,
    stp_representation_item * it)
{
    if (!it)
	return;

    if (it->isa(ROSE_DOMAIN(stp_annotation_plane)))
    {
	// do we do anything special with the plane?
	stp_annotation_plane * ap = ROSE_CAST(stp_annotation_plane, it);
	unsigned i, sz;

	for (i = 0, sz = ap->elements()->size(); i<sz; i++) {
	    // either draughting_callout or styled_item, both of which
	    // are rep items.
	    stp_representation_item * elem =
		ROSE_CAST(stp_representation_item,
		rose_get_nested_object(ap->elements()->get(i)));

	    export_annotation(poly, elem);
	}
    }

    else if (it->isa(ROSE_DOMAIN(stp_annotation_occurrence)))
    {

	stp_annotation_occurrence * ao
	    = ROSE_CAST(stp_annotation_occurrence, it);

	if (!ao->item()) return;

	if (ao->item()->isa(ROSE_DOMAIN(stp_geometric_set))) {
	    export_annotation(poly, ROSE_CAST(stp_geometric_set, ao->item()));
	}
    }

    else {
	stixmesh_ec()->warning("export_annotation unimplemented case: %s\n",
	    it->domain()->name());
    }

}

static void export_model_body(
    Value& exp,
    stp_representation * model)
{
    char buff[20];
    add_ref_id(model, buff);
    exp["id"] = buff;

    SetOfstp_representation_item * items = model->items();
    unsigned sz = items->size();
    for (unsigned i = 0; i<sz; i++) {
	Value poly;
	stp_representation_item * it = items->get(i);
	export_annotation(poly, it);
	if (poly != Value::null && sz > 1)
	    exp["polylines"].append(poly);
	else if (poly != Value::null)
	    exp["polylines"] = poly;
    }
}

static void export_model(
    Value& expmodel,
    stp_representation * model,
    const char * fn)
{
    if (model->wasVisited())
	return;
    model->visit();

    const char * out_dir = fn;

    if (!out_dir)
	export_model_body(expmodel, model);
    else {

	char id[100];
	sprintf(id, "id%lu", model->entity_id());

	RoseStringObject fname = "annotation_";
	fname.cat(id);
	fname.cat(".JSON");

	char idch[100];
	sprintf(idch, "annotation_%s.JSON", id);

	char buff[20];
	add_ref_id(model, buff);
	expmodel["id"] = buff;

	expmodel["href"] = idch;

	FILE * fd = open_dir_file(fn, fname);

	Value exportVal;
	export_model_body(exportVal["annotation"], model);
	
	StyledWriter writer;
	//FastWriter writer;
	std::string output = writer.write(exportVal);
	const char * c = output.c_str();
	
	if (fd != NULL){
	    fputs(c, fd);
	    fclose(fd);
	}
    }
}

static void export_rep_item(
    Value& bod,
    stp_representation * rep,
    stp_representation_item * it)
{
    if (!it)
	return;

    if (it->isa(ROSE_DOMAIN(stp_bounded_curve))) {
	append_step_curve(bod , ROSE_CAST(stp_bounded_curve, it));
    }

}

static void export_constructive_geom_body(
    Value& expcg,
    stp_constructive_geometry_representation * cgr)
{
    char buff[20];
    add_ref_id(cgr, buff);
    expcg["id"] = buff;

    SetOfstp_representation_item * items = cgr->items();
    unsigned sz = items->size();
    for (unsigned i = 0; i<sz; i++) {
	Value bod;
	stp_representation_item * it = items->get(i);
	export_rep_item(bod, cgr, it);
	expcg["polyline"].append(bod);
    }
}

static void export_constructive_geom(
    Value& expcg,
    stp_constructive_geometry_representation * cg,
    const char * fn)
{
    if (cg->wasVisited())
	return;
    cg->visit();

    if (!fn)
	export_constructive_geom_body(expcg, cg);
    else {

	char id[100];
	sprintf(id, "id%lu", cg->entity_id());

	RoseStringObject fname = "constructive_";
	fname.cat(id);
	fname.cat(".JSON");

	char buff[20];
	add_ref_id(cg, buff);
	expcg["id"] = buff;

	char id2[64];
	sprintf(id2, "%s", fname);
	expcg["href"] = id2;


	FILE * fd = open_dir_file(fn, fname);

	Value exportVal;
	export_constructive_geom_body(exportVal["annotation"], cg);


	StyledWriter writer;
	//FastWriter writer;
	std::string output = writer.write(exportVal);
	const char * c = output.c_str();

	if (fd != NULL){
	    fputs(c, fd);
	    fclose(fd);
	}
    }

}

static void append_annotations(
    Value& json,
    stp_representation * sr,
    const char * fn)
{
    if (!sr->isa(ROSE_DOMAIN(stp_shape_representation)))
	return;

    StixMeshRepresentationVec * models = stixmesh_get_draughting_models(
	ROSE_CAST(stp_shape_representation, sr));

    StixMeshConstructiveGeomVec * cgeom = stixmesh_get_constructive_geometry(sr);

    if (!models && !cgeom)
	return;

    if (models) {
	unsigned sz = models->size();

	for (unsigned i = 0; i<sz; i++) {
	    Value expmodel;
	    stp_representation * model = models->get(i);
	    export_model(expmodel, model,fn);
	    json["annotations"].append(expmodel);
	}
    }

    if (cgeom) {
	unsigned sz = cgeom->size();

	for (unsigned i = 0; i<sz; i++) {
	    Value expcg;
	    stp_constructive_geometry_representation * cg = cgeom->get(i);
	    export_constructive_geom(expcg, cg,fn);
	    json["annotations"].append(expcg);
	}
    }
}

static void append_facet(
    const StixMeshFacetSet * fs,
    unsigned fidx,
    int write_normal,
    Value &facets
    )
{
    const StixMeshFacet * f = fs->getFacet(fidx);
    if (!f) return;

    Value vertices;
    vertices.append(f->verts[0]);
    vertices.append(f->verts[1]);
    vertices.append(f->verts[2]);
    facets["v"] = vertices;

    if (write_normal) {
	// facet_normal_now_computed_in_latest_versions
#ifdef LATEST_STDEV
	double fnorm[3];
	fs->getFacetNormal(fnorm, fidx);
#else
	const double * fnorm = fs->getNormal(f->facet_normal);
#endif
	Value fnorms;
	fnorms.append(fnorm[0]);
	fnorms.append(fnorm[1]);
	fnorms.append(fnorm[2]);
	facets["fn"] = fnorms;
    }

    Value normals;
    for (unsigned j = 0; j<3; j++) {
	// facet_normal_now_computed_in_latest_versions
#ifdef LATEST_STDEV
	const double * normal = fs->getNormal(f->normals[j]);
#else
	const double * normal = fs->getNormal(f->vert_normals[j]);
#endif
	if (normal)
	{
	    Value sing_norm;
	    sing_norm["d"].append(normal[0]);
	    sing_norm["d"].append(normal[1]);
	    sing_norm["d"].append(normal[2]);
	    normals.append(sing_norm);
	}
    }
    facets["n"] = normals;
}

void append_shell_facets(
    const StixMeshStp * shell,
    Value &shellVal
    )
{
    int WRITE_NORMAL = 0;
    unsigned i, sz;
    unsigned j, szz;
    const StixMeshFacetSet * facets = shell->getFacetSet();

    char id[64];
    add_ref_id(shell->getStepSolid(),id);
    shellVal["id"] = id;

    unsigned dflt_color = stixmesh_get_color(shell->getStepSolid());
    if (dflt_color != STIXMESH_NULL_COLOR){
	char buff[] = "rrggbb ";
	sprintf(buff, "%06x", dflt_color);
	shellVal["color"] = buff;
    }
    //std::cout << "I am inside append_shell_facets" << std::endl;
    //system("pause");
    Value verts;
    for (i = 0, sz = facets->getVertexCount(); i<sz; i++)
    {
	const double * pt = facets->getVertex(i);
	Value point;
	Value point2;
	point2.append(pt[0]);
	point2.append(pt[1]);
	point2.append(pt[2]);
	point["p"] = point2;
	verts.append(point);
    }

    Value vert_arr;
    vert_arr["v"] = verts;
    shellVal["verts"] = vert_arr;

    // The facet set has all of the facets for the shell.  Break it up
    // into groups by step face.

    const char * color2;
    for (i = 0, sz = shell->getFaceCount(); i<sz; i++)
    {
	Value facets2;
	const StixMeshStpFace * fi = shell->getFaceInfo(i);
	unsigned first = fi->getFirstFacet();
	unsigned color = stixmesh_get_color(fi->getFace());
	if (first == ROSE_NOTFOUND)
	    continue;

	// Always tag the face with a color, unless everything is null. 
	if (color == STIXMESH_NULL_COLOR)
	    color = dflt_color;

	if (color != STIXMESH_NULL_COLOR){
	    char buff2[] = "rrggbb ";
	    sprintf(buff2, "%06x", dflt_color);
	    color2 = buff2;
	    facets2["color"] = color2;
	}
	for (j = 0, szz = fi->getFacetCount(); j<szz; j++) {
	    Value tempfacet;
	    append_facet(facets, j + first, WRITE_NORMAL, tempfacet);
	    facets2["f"].append(tempfacet);
	}
	shellVal["facets"].append(facets2);
    }
}

static void export_shell(
    const StixMeshStp * shell,
    Value &shell_arr,
    const char * fn
    )
{
    if (!shell) return;

    Value shellVal;
    if (!fn) {
	append_shell_facets(shell, shellVal);
	shell_arr.append(shellVal);
    }
    else
    {
	unsigned i, sz;
	StixMeshBoundingBox bbox;
	const StixMeshFacetSet * facets = shell->getFacetSet();

	// compute the bounding box for the shell
	for (i = 0, sz = facets->getVertexCount(); i<sz; i++)
	{
	    const double * pt = facets->getVertex(i);
	    bbox.update(pt);
	}

	char fname[100];
	sprintf(fname, "shell_id%lu.JSON", shell->getStepSolid()->entity_id());

	char id[64];
	add_ref_id(shell->getStepSolid(),id);
	shellVal["id"] = id;
	shellVal["size"] = facets->getFacetCount();

	Value bboxArr;
	//Turn the 6 bbox values into an array and set equal to bbox
	bboxArr.append(bbox.minx);
	bboxArr.append(bbox.miny);
	bboxArr.append(bbox.minz);
	bboxArr.append(bbox.maxx);
	bboxArr.append(bbox.maxy);
	bboxArr.append(bbox.maxz);

	shellVal["bbox"] = bboxArr;

	// append the area 
	double area = 0.;
	for (i = 0, sz = shell->getFaceCount(); i<sz; i++) {
	    const StixMeshStpFace * face = shell->getFaceInfo(i);
	    area += face->getArea();
	}

	shellVal["area"] = area;
	shellVal["href"] = fname;

	shell_arr.append(shellVal);

	FILE * fd = open_dir_file(fn, fname);

	Value exportVal;
	Value exportVal2;
	append_shell_facets(shell, exportVal);
	exportVal2["shell"] = exportVal;

	StyledWriter writer;
	//FastWriter writer;

	std::string output = writer.write(exportVal2);
	const char * c = output.c_str();
	fputs(c, fd);

	fclose(fd);
    }
}

void reportProgress(stp_representation_item * it)
{
    char buff[64];
    sprintf(buff, "Finished facet: shell #%lu", it->entity_id());
}

int flushCompletedShells(int wait, StixMeshStpAsyncMaker& async, Value &shellArr, const char * fn)
{
    StixMeshStp * mesh;

    while ((mesh = async.getResult(wait)) != 0) {
	reportProgress(mesh->getStepSolid());
	export_shell(mesh, shellArr, fn);
	delete mesh;
    }

    return async.hasRunningJobs();
}

void dumpShapeAsm(
    Value& json,
    Value& shape,
    Value& shellArr,
    stp_representation * sr,
    rose_uint_vector * roots,
    const char * fn,
    StixMeshStpAsyncMaker& async,
    StixMeshOptions * opts)
{
    unsigned i, j, sz;

    if (sr->wasVisited()) return;
    sr->visit();

    StixMgrAsmShapeRep * srm = StixMgrAsmShapeRep::find(sr);
    if (!srm) {
	stixmesh_ec()->error("Could not find RM\n");
	return;
    }

    int found;

    if (roots == 0)
	found = ROSE_TRUE;
    else {
	found = ROSE_FALSE;
	unsigned sz = roots->size();

	unsigned eid = sr->entity_id();

	for (i = 0; i<sz; i++) {
	    unsigned e = roots->get(i);
	    if (e == eid) {
		found = ROSE_TRUE;
		break;
	    }
	}
    }

    char buff[20];
    add_ref_id(sr, buff);
    shape["id"] = buff;

    StixUnit unit = stix_get_context_length_unit(sr);
    if (unit != stixunit_unknown) {
	char buff[20];
	sprintf(buff, "%s %f", stix_get_unit_name(unit), stix_get_converted_measure(1., unit, stixunit_m));
	shape["unit"] = buff;
    }

    if (found)
	appendShellRefs(shape, sr);

    append_annotation_refs(shape, sr);

    for (j = 0, sz = srm->child_rels.size(); j < sz; j++){
	Value temp_shape;
	append_asm_child(temp_shape, srm->child_rels[j]);
	if (temp_shape != Value::null)
	    shape["child"].append(temp_shape);
    }

    for (j = 0, sz = srm->child_mapped_items.size(); j < sz; j++){
	Value temp_shape;
	append_asm_child(temp_shape, srm->child_mapped_items[j]);
	if (temp_shape != Value::null)
	    shape["child"].append(temp_shape);
    }

    for (j = 0, sz = srm->child_rels.size(); j < sz; j++){
	Value childshape;
	dump_shape_child(json, childshape, shellArr, srm->child_rels[j], roots,fn,async,opts);
	if (childshape != Value::null)
	    json["shape"].append(childshape);
    }

    for (j = 0, sz = srm->child_mapped_items.size(); j < sz; j++){
	Value childshape;
	dump_shape_child(json, childshape, shellArr, srm->child_mapped_items[j], roots,fn,async,opts);
	if (childshape != Value::null)
	    json["shape"].append(childshape);
    }

    if (found) {
	appendShells(sr,async,opts);
	flushCompletedShells(1, async, shellArr, fn);
	append_annotations(json,sr,fn);
    }
}

void dump_shapes(
    Value &json,
    Value & shellArr,
    StpAsmShapeRepVec * shapes,
    const char * fn,
    StixMeshStpAsyncMaker& async,
    StixMeshOptions * opts)
{
    for (unsigned i = 0; i < shapes->size(); i++){
	Value shape;
	dumpShapeAsm(json, shape, shellArr, shapes->get(i), 0, fn, async, opts);
	if (shape != Value::null)
	    json["shape"].append(shape);
    }
}

/**********************************************************************************/

void dump_exec_atts(
    StpAsmShapeRepVec * shapes, Value& json,
    Executable_IF * exec, RoseBoolean show_fixture)
{
    char buff[20];
    add_ref_id(exec->getRootObject(), buff);
    json["id"] = buff;
    json["name"] =  exec->get_its_id();

    const char * enabled = exec->get_enabled();

    if (enabled && !strcmp(enabled, "disabled"))
	json["enabled"] = "false";
    else
	json["enabled"] = "true";
    
    stp_product_definition * to_be_prod = exec->get_to_be_geometry();
    stp_shape_representation * to_be = get_shape(to_be_prod);
    if (to_be) {
	char buff[20];
	add_ref_id(to_be, buff);
	json["to_be"] = buff;
	shapes->append(to_be);
    }    
    
    stp_shape_representation * as_is = get_shape(exec->get_as_is_geometry());

    if (!as_is && to_be_prod) {
	/* Legacy code for the the rawpiece in the to_be.  I don't know if this
	 * is necessary, but here it is.
	 */
	Workpiece * wp = Workpiece::find(to_be_prod);
	if (wp) 
	    as_is = get_shape(wp->get_its_rawpiece());
    }
    
    if (as_is) {
	char buff[20];
	add_ref_id(as_is, buff);
	json["as_is"] = buff;
	shapes->append(as_is);
    }

    if (show_fixture) {
	stp_shape_representation * fixture
	    = get_shape(exec->get_fixture_geometry());
	if (fixture) {
	    char buff[20];
	    add_ref_id(fixture, buff);
	    json["fixture"] = buff;
	    shapes->append(fixture);
	}
    }
}


static Value get_shape_refs(StpAsmShapeRepVec * shapes)
{
    Value ids;
    bool b = false;
    if (shapes == 0 || shapes->size() == 0)
	return " ";
    if (shapes->size() > 1)
	b = true;

    for (unsigned i=0; i<shapes->size(); i++) {
	char buff[20];
	add_ref_id(shapes->get(i), buff);
	if (b == false)
	    ids = buff;
	else
	    ids.append(buff);
    }
    return ids;
}

void resolve_workpiece(StpAsmShapeRepVec * as_is, StpAsmShapeRepVec * to_be,
		       stp_product_definition * pd)
{
    Workpiece * wp = Workpiece::find(pd);
    if (!wp) {
	return;
    }

    stp_shape_representation * geo = wp->get_its_geometry();
    if (geo)
	to_be->append(geo);

    Workpiece * raw = Workpiece::find(wp->get_its_rawpiece());
    if (raw) {
	geo = raw->get_its_geometry();
	if (geo)
	    as_is->append(geo);
    }
}			   



static void dump_setup(
    Value& json,
    Value& shellArr,
    stp_product_definition_formation * aim_setup,
    const char * fn,
    StixMeshStpAsyncMaker& async,
    StixMeshOptions * opts)
{    
    if (aim_setup->wasVisited())
	return;

    aim_setup->visit();
    
    Setup * setup = Setup::find(aim_setup);
    if (!setup) {
	printf ("Could not get AIM setup\n");
	exit (2);
    }

    stp_machining_fixture_usage * aim_usage = setup->get_its_fixture_usage();

    stp_shape_representation * shape =0;
    
    char buff[20];
    add_ref_id(aim_setup, buff);
    json["setup"]["id"] = buff;
    add_xform(json["setup"], "origin", setup->get_its_origin());

    if (aim_usage) {
	Fixture_usage * usage = Fixture_usage::find(aim_usage);

	stp_product_definition * prod = usage->get_its_product();
	shape = get_shape(prod);

	add_xform(json["setup"], "mount_ref", usage->get_mount_reference());
	add_xform(json["setup"], "workpiece_ref",
		  usage->get_workpiece_reference());
	char buff[20];
	add_ref_id(shape,buff);
	json["setup"]["geometry"] = buff;
    }

    if (shape){
	Value shapeJ;
	dumpShapeAsm(json, shapeJ, shellArr, shape, 0,fn, async, opts);
	json["shape"].append(shapeJ);
    }
}


void dump_exec(
    Value& json,
    Value& shellArr,
    stp_machining_process_executable * exec,
    RoseBoolean show_fixture,
    const char * fn,
    StixMeshStpAsyncMaker& async,
    StixMeshOptions * opts)
{    
    if (exec->wasVisited())
	return;

    exec->visit();

    StpAsmShapeRepVec shapes;

    {
	Workplan * wp = Workplan::find(exec);

	if (wp) {
	    unsigned sz = wp->size_its_elements();
	    unsigned i;
	    Value workplan;

	    stp_product_definition_formation * setup = wp->get_its_setup();

	    show_fixture = (setup == 0);
	    
	    dump_exec_atts(&shapes, workplan, wp, show_fixture);

	    if (setup){
		char buff[20];
		add_ref_id(setup, buff);
		workplan["setup"] = buff;
	    }
	    
	    if (sz > 0) {
		Value ids;
		for (i=0; i<sz; i++) {		    
		    char buff[20];
		    add_ref_id(wp->get_its_elements(i)->getValue(), buff);
		    ids.append(buff);
		}
		workplan["elements"] = ids;
	    }
	    workplan["type"] = "Workplan";
	    json["Workplan"].append(workplan);

	    if (setup)
		dump_setup(json, shellArr, setup,fn,async,opts);
	    
	    for (i=0; i<sz; i++) {
		stp_machining_process_executable * ex =
		    wp->get_its_elements(i)->getValue();
		dump_exec(json, shellArr, ex, show_fixture,fn, async, opts);
	    }

	    dump_shapes(json, shellArr, &shapes,fn, async, opts);
	    
	    return;
	}
    }

    {
	Machining_workingstep * ws = Machining_workingstep::find(exec);
	if (ws) {
	    
	    Value workingstep;
	    
	    dump_exec_atts(&shapes, workingstep, ws, show_fixture);

	    stp_machining_operation * op = ws->get_its_operation();
	    stp_axis2_placement_3d * placement = ws->get_toolpath_orientation();
	    
	    if (op){
		char buff[20];
		add_ref_id(op, buff);
		workingstep["op"] = buff;
	    }
	    if (placement){
		char buff[20];
		add_ref_id(placement, buff);
		workingstep["orientation"] = buff;
	    }
	    workingstep["type"] = "Workingstep";
	    json["Workingstep"].append(workingstep);

	    printf ("dumping op:%p\n",op);
	    
	    Value operation;
	    dump_operation(json, shellArr, operation, op, fn,async,opts);
	    if (operation != Value::null)
		json["operation"].append(operation);

	    printf ("Dumping placement:%p\n", placement);
	    
	    Value placementJ;
	    dump_placement (placementJ, placement);
	    if (placementJ != Value::null)
		json["placement"].append(placementJ);

	    dump_shapes(json, shellArr, &shapes, fn, async, opts);
	    return;
	}
    }

    {
	Selective * sel = Selective::find(exec);
	if (sel) {
	    unsigned i;
	    Value selective;
	    dump_exec_atts(&shapes, selective, sel, show_fixture);

	    unsigned sz = sel->size_its_elements();
	    Value ids;
	    for (i=0; i<sz; i++) {
		char buff[20];
		add_ref_id(sel->get_its_elements(i)->getValue(), buff);
		ids.append(buff);
	    }
	    selective["elements"] = ids;
	    selective["type"] = "selective";
	    json["selective"].append(selective);

	    for (i=0; i<sz; i++) {
		dump_exec(json, shellArr, sel->get_its_elements(i)->getValue(),
			  show_fixture,fn,async, opts);
	    }

	    dump_shapes(json, shellArr, &shapes,fn,async,opts);
	    return;
	}
    }

    {
	Frame_definition_workingstep * ws
	    = Frame_definition_workingstep::find(exec);
	if (ws) {
	    Value fdws;
	    printf ("Have FDW\n");

	    char buff[20];
	    add_ref_id(ws->getRootObject(), buff);
	    fdws["id"] = buff;
	    fdws["name"] = ws->get_its_id();
	    fdws["type"] = "frame_definition_workingstep";
	    json["frame_definition_workingstep"].append(fdws);

	    return;
	}
    }

    {
	Compensation_workingstep * ws
	    = Compensation_workingstep::find(exec);
	if (ws) {
	    Value compws;
	    char buff[20];
	    add_ref_id(ws->getRootObject(), buff);
	    compws["id"] = buff;
	    compws["name"] = ws->get_its_id();
	    compws["type"] = "compensation_workingstep";
	    json["compensation_workingstep"].append(compws);
	    
	    return;
	}
    }
    
    printf ("resolve_exec: could not find exec: #%d (%s)\n", exec->entity_id(),
	    exec->domain()->name());
    exit (2);
}




void dump_project(
    Value& json,
    Value& shellArr,
    Project * proj,
    const char * fn,
    StixMeshStpAsyncMaker& async,
    StixMeshOptions * opts)
{
    unsigned i,sz;

    stp_machining_workplan * wp = proj->get_main_workplan();
    
    if (!wp) {
	printf ("No main workplan\n");
	exit (2);
    }

    StpAsmShapeRepVec as_is;
    StpAsmShapeRepVec to_be;
    
    sz = proj->size_its_workpieces();
    for (i=0; i<sz; i++) {
	stp_product_definition * pd = proj->get_its_workpieces(i)->getValue();
	resolve_workpiece(&as_is, &to_be, pd);
    }

    char buff[20];
    add_ref_id(wp, buff);
    json["project"]["wp"] = buff;


    Value current = get_shape_refs(&as_is);
    Value will_be = get_shape_refs(&to_be);
    
    if (current != " ")
	json["project"]["as_is"] = current;
    if (will_be != " ")
	json["project"]["to_be"] = will_be;

    dump_shapes(json, shellArr, &as_is,fn,async,opts);

    dump_shapes(json, shellArr, &to_be,fn,async,opts);
        
    dump_exec (json, shellArr, wp, ROSE_TRUE, fn,async,opts);
}

static StixUnit get_unit(stp_product_definition * pd)
{
    unsigned i;
    StpAsmShapeRepVec as_is;
    StpAsmShapeRepVec to_be;

    printf ("Resolving unit: %p\n", pd);
    if (pd == 0)
	return stixunit_unknown;
    
    resolve_workpiece(&as_is, &to_be, pd);
    printf ("Have WP: %d %d\n", as_is.size(), to_be.size());
    
    for (i=0; i<as_is.size(); i++) {
	StixUnit unit = stix_get_context_length_unit(as_is[i]);
	if (unit != stixunit_unknown)
	    return unit;
    }

    for (i=0; i<to_be.size(); i++) {
	StixUnit unit = stix_get_context_length_unit(to_be[i]);
	if (unit != stixunit_unknown)
	    return unit;
    }
    return stixunit_unknown;
}

static StixUnit get_unit(Executable_IF * exec)
{
    StixUnit ret;
    
    ret = get_unit(exec->get_to_be_geometry());
    if (ret != stixunit_unknown)
	return ret;

    ret = get_unit(exec->get_as_is_geometry());
    if (ret != stixunit_unknown)
	return ret;

    Workplan_IF * wp = ARM_CAST(Workplan_IF, exec);
    if (wp) {
	unsigned sz = wp->size_its_elements();
	for (unsigned i=0; i<sz; i++) {
	    Executable_IF * el = Executable_IF::find(
		wp->get_its_elements(i)->getValue());
	    ret = get_unit(el);
	    printf ("Have unit: %d\n", ret);
	    
	    if (ret != stixunit_unknown)
		return ret;
	}
    }
    
    return stixunit_unknown;
}

static StixUnit get_unit(Project * proj)
{
    unsigned i,sz;
    StixUnit unit;
    
    stp_machining_workplan * wp = proj->get_main_workplan();

    if (!wp) {
	printf ("No main workplan\n");
	exit (2);
    }

    StpAsmShapeRepVec as_is;
    StpAsmShapeRepVec to_be;
    
    sz = proj->size_its_workpieces();
    for (i=0; i<sz; i++) {
	stp_product_definition * pd = proj->get_its_workpieces(i)->getValue();
	resolve_workpiece(&as_is, &to_be, pd);
    }

    for (i=0; i<as_is.size(); i++) {
	unit = stix_get_context_length_unit(as_is[i]);
	if (unit != stixunit_unknown)
	    return unit;
    }

    for (i=0; i<to_be.size(); i++) {
	unit = stix_get_context_length_unit(to_be[i]);
	if (unit != stixunit_unknown)
	    return unit;
    }

    unit = get_unit(Workplan::find(proj->get_main_workplan()));
    if (unit != stixunit_unknown)
	return unit;
    
    printf ("No unit\n");
    exit (2);
}

static StixUnit get_unit(RoseDesign * des)
{
    ARMCursor cur;
    cur.traverse(des);
    cur.domain(Project::type());

    Project * proj;
    
    while ((proj = ARM_CAST(Project, cur.next())) != 0) {
	StixUnit ret = get_unit(proj);
	if (ret != stixunit_unknown)
	    return ret;
    }
    
    printf ("Could not get unit\n");
    exit (2);
    
}


/*
 * usage: stpfile JSONfile
 */
int NCtoJSON(stp2webgl_opts * opts)
{
    /* Disable buffering */
    setvbuf(stdout, 0, _IONBF, 2);

    const char * fname = 0;
    const char * JSONname = 0;


    RoseBoolean write_dir = ROSE_FALSE;    
    const char * out_dir = 0;    

    if (opts->do_split){
	write_dir = ROSE_TRUE;
	fname = opts->srcfile;
    }
    else
	fname = opts->srcfile;

    FILE * out =0;
    char o_d[64];

    if (!write_dir) {
	if (!JSONname)
	    out = fopen("YourStepFile.JSON", "w");
	else
	   out = fopen (JSONname, "w"); 
    }
    else {
	if (!JSONname)
	    JSONname = "Full_JSON_Moldy";
	out_dir = JSONname;
	strcpy(o_d, out_dir);
	if (mkdir(out_dir MKDIR_PROT_PARM) && !(DIR_ALREADY_EXISTS)) {
	    printf ("Cannot create directory %s\n", out_dir);
	    exit (2);
	}
	
	RoseStringObject path = out_dir;
	path.cat ("/index.JSON");
	out = fopen(path, "w");
    }

    printf ("Loaded\n");

    stix_tag_units(opts->design);

    stix_tag_asms(opts->design);
    stixmesh_resolve_presentation(opts->design);
    ARMpopulate(opts->design);

    ARMCursor cur;
    cur.traverse(opts->design);
    cur.domain(Project::type());

    Project * proj;

    Value json;

    StixUnit unit = get_unit(opts->design);

    char buff[20];
    sprintf (buff, "%s %f", stix_get_unit_name(unit),stix_get_converted_measure(1., unit, stixunit_m));

    json["project-data"]["unit"] = buff;
    
    ROSE.beginTraversal();

    printf ("Dumping project\n");
    StixMeshStpAsyncMaker async;
    StixMeshOptions * opts2 = 0;
 
    Value shellArray;
    while ((proj = ARM_CAST(Project, cur.next())) != 0) {
	if (write_dir)
	    dump_project(json["project-data"], shellArray, proj, o_d, async, opts2);
	else
	    dump_project(json["project-data"], shellArray, proj, 0, async, opts2);
    }
    if (shellArray != Value::null)
	json["project-data"]["shell"] = shellArray;

    ROSE.endTraversal();

    //FastWriter writer;
    StyledWriter writer;
    std::string output = writer.write(json);
    const char * c = output.c_str();
    fputs(c, out);

    fclose(out);

    return 0;
}
