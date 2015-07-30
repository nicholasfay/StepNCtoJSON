/* $RCSfile: dump_project.cxx,v $
 * $Revision: 1.17 $ $Date: 2014/03/26 21:24:59 $
 * Auth: Jochen Fritz (jfritz@steptools.com)
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
 *  Convert a STEP-NC file to XML for WebGL comsumption.
 */


#include <math.h>
#include <stp_schema.h>
#include <stix.h>
#include <cstdlib>
#include <iostream>

#include <RoseXMLWriter.h>
#include "stixmesh_writer.h"
#include "stp2webgl.h"

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


stp_product_definition * stmfg_get_tool_product(
    stp_machining_operation * op
    );

stp_product_definition * stmfg_get_tool_product(
    stp_machining_tool * mt
    );


/************************************************/
static void add_ref_id (RoseXMLWriter * xml, RoseObject * obj)
{
    char buff[20];
    sprintf (buff, "id%d", obj->entity_id());
    
    xml->text(buff);
}


static void add_ref (RoseXMLWriter * xml, const char * att, RoseObject * obj)
{
    char buff[20];
    sprintf (buff, "id%d", obj->entity_id());
    
    xml->addAttribute(att, buff);
}

static void add_id (RoseXMLWriter * xml, RoseObject * obj)
{
    add_ref(xml, "id", obj);
}


/************************************************/

static stp_shape_representation * get_workpiece_geometry(stp_product_definition * pd)
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



static double get_tech_feedrate(stp_machining_technology * tech, StixUnit len_unit)
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


static void append_vec3(RoseXMLWriter * xml, double v[3])
{
    char buff[20];
    for (int i=0; i<3; i++) {
	if (i > 0)
	    xml->text(" ");
	sprintf (buff, "%g", v[i]);
	xml->text(buff);
    }
}


static void write_tp_point(
    StixMeshXMLExport * exp,
    RoseXMLWriter * xml,
    TPState * st)
{
    double u = st->u;
    double xyz[3];
    st->bcurve.eval(xyz, u);
    
    xml->beginElement("p");
    exp->appendVal(xml, "u", st->u);
    
    if (st->d != 0.) {
	exp->appendVal(xml, "d", st->d);
    }
    
    if (st->t != 0.) {
	exp->appendVal(xml, "t", st->t);
    }

    xml->beginAttribute ("l");
    //   fprintf (out, "l=\"%f %f %f", xyz[0], xyz[1], xyz[2]); 
    append_vec3(xml, xyz);
   
    if (!st->axis.isEmpty()) {
	double ijk[3];
	st->axis.eval(ijk, u);
	//fprintf (out, " %f %f %f", ijk[0], ijk[1], ijk[2]);
	xml->text(" ");
	append_vec3(xml, ijk);
    }

    xml->endAttribute();
    
    xml->endElement("p");
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
    StixMeshXMLExport * exp, 
    RoseXMLWriter * xml,
    TPState * st,
    double u)
{
    double old_u = st->u;
    double len = st->bcurve.arcLength(old_u, u);

    st->d += len;
    if (st->s != 0)
	st->t += len/st->s;
    
    st->u = u;
    write_tp_point(exp, xml, st);
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
    StixMeshXMLExport * exp,
    RoseXMLWriter * xml,
    stp_machining_technology * tech,
    StixUnit len_unit)
{
    if (tech->wasVisited())
	return;

    tech->visit();
    
    {
	Milling_technology * arm = Milling_technology::find(tech);
	if (arm) {
	    xml->beginElement("milling_technology");
	    exp->appendRef(xml, "id", tech);

	    if (arm->isset_feedrate()) {

		exp->appendVal(xml, "feedrate",
			       get_tech_feedrate(tech, len_unit));
	    }

	    if (arm->isset_spindle()) {
		exp->appendVal(xml, "spindle",
					get_rot_value(arm->get_spindle()));
	    }

	    xml->endElement("milling_technology");
	    
	    return;
	}
    }

    printf ("Unimplemented technology case: #%d\n", tech->entity_id());
    exit (2);

}


static void append_double(RoseXMLWriter * xml, double val)
{
    char buff[20];
    sprintf (buff, "%g", val);
    xml->text(buff);
}


static void append_bbox(
    RoseXMLWriter * xml,
    const char * name,
    StixMeshBoundingBox * bbox)
{
    xml->beginAttribute(name);

    append_double(xml, bbox->minx);
    xml->text(" ");
    append_double(xml, bbox->miny);
    xml->text(" ");
    append_double(xml, bbox->minz);
    xml->text(" ");
    append_double(xml, bbox->maxx);
    xml->text(" ");
    append_double(xml, bbox->maxy);
    xml->text(" ");
    append_double(xml, bbox->maxz);
    
    xml->endAttribute();
}


static FILE * open_dir_file(const char * dir, const char * fname)
{
    RoseStringObject path = dir;
    path.cat("/");
    path.cat(fname);

    return fopen(path, "w");
}


static void dump_toolpath(
    StixMeshXMLExport * exp,
    Trajectory_IF * tp,
    stp_bounded_curve * bcurve,
    stp_representation * bcurve_rep,
    stp_bounded_curve * axis,
    stp_representation * axis_rep,
    double tol)
{
    stp_machining_technology * tech = tp->get_its_technology();    
    RoseXMLWriter * tp_xml = exp->getMainXML();

    StixUnit lun = stix_get_context_unit(bcurve_rep, stixvalue_length);

    double feed = get_tech_feedrate(tech, lun);

    const char * out_dir = exp->getOutputDir();

    
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
    RoseXMLWriter * xml = exp->getMainXML();
    
    if (out_dir) {

	char id[100];
	sprintf (id, "id%lu", tp->getRootObject()->entity_id());

	RoseStringObject fname = "toolpath_";
	fname.cat(id);
	fname.cat (".xml");

	xml->beginElement("toolpath");
	xml->addAttribute("id", id);

	char sz_buff[10];
	sprintf (sz_buff, "%u", u_vals.size());
	xml->addAttribute("size", sz_buff);

	append_bbox(xml, "bbox", &bbox);

	xml->addAttribute("href", fname);

	// creating a temp writer
	FILE * fd = open_dir_file(out_dir, fname);
	RoseOutputFile * stream = new RoseOutputFile(fd, fname);

	tp_xml = new RoseXMLWriter(stream);
	tp_xml->escape_dots = ROSE_FALSE;
	tp_xml->writeHeader();
    } 

    
    tp_xml->beginElement ("toolpath");
    exp->appendRef(tp_xml, "id", tp->getRootObject());


    /* We may have a degenerate toolpath.  If so, output nothing */

    if (u_vals.size() > 0.) {
	stat.bcurve.extractTolerancedPoints(&u_vals, tol, 1);

	double prev_u = u_vals[0] ;

	stat.u = u_vals[0];
	write_tp_point(exp, tp_xml, &stat);

	for (unsigned i=1; i<u_vals.size(); i++) {

	    double u = u_vals[i];

	    rose_real_vector pushed_points_axis;
	    
	    if (axis) {
		insert_axis_points(&pushed_points_axis, &stat.axis,
				   prev_u, u);
	    }
	    
	    for (unsigned j=0; j<pushed_points_axis.size(); j++) {
		append_tp_point(exp, tp_xml, &stat, pushed_points_axis[j]);
	    }
	    
	    append_tp_point(exp, tp_xml, &stat, u);
	    
	    prev_u = u;
	}
    }
    
    tp_xml->endElement("toolpath");


    if (out_dir) {
	// delete and close temporary writer.
	RoseOutputFile * stream = (RoseOutputFile*) tp_xml-> stream();
	FILE * fd = stream-> as_file();

	tp_xml->close();
	stream-> flush();

	delete tp_xml;
	delete stream;
	fclose (fd);

	char buff[30];
	sprintf (buff, "%f", stat.d);
	xml->addAttribute("length", buff);

	xml->endElement("shell");
    }


    RoseXMLWriter * main_xml = exp->getMainXML();
    
    dump_tech(exp, main_xml, tech, lun);
    
}


static void dump_toolpath(
    StixMeshXMLExport * st,
    Cutter_location_trajectory * tp,
    double tol)

{
    stp_bounded_curve * bcurve = tp->get_basiccurve();
    stp_representation * bcurve_rep = tp->get_basiccurve_rep();    
    stp_bounded_curve * axis = tp->get_its_toolaxis();
    stp_representation * axis_rep = 0;
    
    if (axis)
	axis_rep = tp->get_its_toolaxis_rep();

    dump_toolpath(st, tp, bcurve, bcurve_rep, axis, axis_rep, tol);
}


static void dump_toolpath(
    StixMeshXMLExport * exp,
    Cutter_contact_trajectory * tp,
    double tol)
{
    stp_bounded_curve * bcurve = tp->get_basiccurve();
    stp_representation * bcurve_rep = tp->get_basiccurve_rep();    
    stp_bounded_curve * axis = tp->get_its_toolaxis();
    stp_representation * axis_rep = 0;
    
    if (axis)
	axis_rep = tp->get_its_toolaxis_rep();

//    dump_toolpath(xml, out_dir, tp, bcurve, bcurve_rep, axis, axis_rep, tol);
    dump_toolpath(exp, tp, bcurve, bcurve_rep, axis, axis_rep, tol);
    printf ("WARNING: cutter_contact_trajectory: surface normal not yet implemented\n");
    
}


void stgl_xml_dump_toolpath(
    StixMeshXMLExport * st,
    stp_machining_toolpath * tp,
    double tol = ROSE_NULL_REAL)
{
    if (tp->wasVisited())
	return;
    
    tp->visit();
    
    {
	Cutter_location_trajectory * arm_tp
	    = Cutter_location_trajectory::find(tp);

	if (arm_tp) {
	    dump_toolpath(st, arm_tp, tol);
	    return;
	}
    }

    {
	Cutter_contact_trajectory * arm_tp
	    = Cutter_contact_trajectory::find(tp);

	if (arm_tp) {
	    dump_toolpath(st, arm_tp, tol);
	    return;
	}
    }
    
    printf ("stgl_xml_dump_toolpath: Unimplemented toolpath case: #%d\n",
	    tp->entity_id());
    exit(2);
}



static void dump_operation (
    StixMeshXMLExport * exp,    
    stp_machining_operation * op)
{
    if (!op) {
	printf ("No AIM operation\n");
	exit (2);
    }

    if (op->wasVisited())
	return;

    op->visit();

    RoseXMLWriter * main_xml = exp->getMainXML();
    
    Operation_IF * arm_op = Operation_IF::find(op);
    if (!arm_op) {
	printf ("No operation from #%d (%s)\n",
		op->entity_id(), op->domain()->name());
	exit (2);
    }

    double tool_len;
    stp_shape_representation * tool_geom = get_workpiece_geometry(
	get_tool_product(&tool_len, op));
    
    main_xml->beginElement("operation");
    exp->appendRef (main_xml, "id", op);

    unsigned sz = arm_op->size_its_toolpath();
    
    if (sz > 0) {
	main_xml->beginAttribute("toolpath");

	for (unsigned i=0; i<sz; i++) {
	    stp_machining_toolpath * tp
		= arm_op->get_its_toolpath(i)->getValue();
	    if (i > 0) main_xml->text (" ");

	    add_ref_id(main_xml, tp);
	}

	main_xml->endAttribute();
    }

    if (tool_geom) {
	add_ref(main_xml, "tool", tool_geom);
	char buff[20];
	sprintf (buff, "%g", tool_len);
	main_xml->addAttribute("tool_length", buff);
    }

    main_xml->endElement("operation");

    for (unsigned i=0; i<sz; i++) {
	stp_machining_toolpath * tp
	    = arm_op->get_its_toolpath(i)->getValue();

	stgl_xml_dump_toolpath(exp, tp);
    }

    exp->dumpShapeAsm(tool_geom, 0);
}



static void add_xform(RoseXMLWriter * xml, const char * att,
		      stp_axis2_placement_3d * place)
{
    if (place == 0)
	return;

    StixMtrx xform(place);
    
    xml->beginAttribute (att);
    unsigned i,j;
    for (i=0; i<4; i++)
	for (j=0; j<4; j++) {
	    char buff[20];
	    sprintf (buff, "%s%g", (i==0&&j==0)?"":" ", xform.get(j,i));
	    xml->text (buff);
	}    
    xml->endAttribute();
}

/* FIXME: consider using a more compact form for the matrix.
 * ex: it we have an identity transform, onit the transform;
 * if we have just a translation, include just the xyz of the translation.
 */
void dump_placement (
    StixMeshXMLExport * exp,
    RoseXMLWriter * xml, 
    stp_axis2_placement_3d * placement)
{
    if (!placement || placement->wasVisited())
	return;

    placement->visit();
    
    xml->beginElement("placement");
    exp->appendRef (xml, "id" , placement);

    StixMtrx xform(placement);

    xml->beginAttribute ("xform");
    
    unsigned i,j;
    for (i=0; i<4; i++)
	for (j=0; j<4; j++) {
	    char buff[20];
	    sprintf (buff, "%s%g", (i==0&&j==0)?"":" ", xform.get(j,i));
	    xml->text (buff);
	}

    xml->endAttribute();
    xml->endElement("placement");
}

static stp_shape_representation * get_shape(stp_product_definition * pd)
{
    if (pd == 0)
	return 0;
    
    Workpiece * wp = Workpiece::find(pd);
    if (wp == 0)
	return 0;

    return wp->get_its_geometry();
}


void dump_shapes(
    StixMeshXMLExport * exp,        
//    RoseXMLWriter * xml, const char * out_dir,
    StpAsmShapeRepVec * shapes)
{
    for (unsigned i=0; i<shapes->size(); i++)
	exp->dumpShapeAsm(shapes->get(i), 0);
}


void dump_exec_atts(
    StpAsmShapeRepVec * shapes, RoseXMLWriter * xml,
    Executable_IF * exec, RoseBoolean show_fixture)
{
    add_id(xml, exec->getRootObject());
    xml->addAttribute("name", exec->get_its_id());

    const char * enabled = exec->get_enabled();

    if (enabled && !strcmp(enabled, "disabled"))
	xml->addAttribute("enabled", "false");
    else
	xml->addAttribute("enabled", "true");
    
    stp_product_definition * to_be_prod = exec->get_to_be_geometry();
    stp_shape_representation * to_be = get_shape(to_be_prod);
    if (to_be) {
	add_ref(xml, "to_be", to_be);
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
	add_ref(xml, "as_is", as_is);
	shapes->append(as_is);
    }

    if (show_fixture) {
	stp_shape_representation * fixture
	    = get_shape(exec->get_fixture_geometry());
	if (fixture) {
	    add_ref(xml, "fixture", fixture);
	    shapes->append(fixture);
	}
    }
}


static void add_shape_refs(
    RoseXMLWriter * xml, const char * attname, StpAsmShapeRepVec * shapes)
{
    if (shapes == 0 || shapes->size() == 0)
	return;
    
    xml->beginAttribute(attname);
    for (unsigned i=0; i<shapes->size(); i++) {
	if (i > 0) xml->text (" ");	    

	add_ref_id(xml, shapes->get(i));
    }
    xml->endAttribute();    
}



#if 0
static stp_shape_representation * get_shape(stp_product_definition * pd)
{
    Workpiece * wp = Workpiece::find(pd);
    if (!wp) {
	return;
    }

    return wp->get_its_geometry();
    if (geo)
	to_be->append(geo);

    Workpiece * raw = Workpiece::find(wp->get_its_rawpiece());
    if (raw) {
	geo = raw->get_its_geometry();
	if (geo)
	    as_is->append(geo);
    }
}			   
#endif

static void resolve_workpiece(StpAsmShapeRepVec * as_is, StpAsmShapeRepVec * to_be,
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
    StixMeshXMLExport * exp,
    stp_product_definition_formation * aim_setup)
{
    RoseXMLWriter * main_xml = exp->getMainXML();
    
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
    
    main_xml->beginElement("setup");

    add_ref(main_xml, "id", aim_setup);
    add_xform(main_xml, "origin", setup->get_its_origin());

    if (aim_usage) {
	Fixture_usage * usage = Fixture_usage::find(aim_usage);

	stp_product_definition * prod = usage->get_its_product();
	shape = get_shape(prod);

	add_xform(main_xml, "mount_ref", usage->get_mount_reference());
	add_xform(main_xml, "workpiece_ref",
		  usage->get_workpiece_reference());
	add_ref (main_xml, "geometry", shape);
    }
    
    main_xml->endElement("setup");

    if (shape)
	exp->dumpShapeAsm(shape,0 );
}


void dump_exec(
    StixMeshXMLExport * exp,    
    stp_machining_process_executable * exec,
    RoseBoolean show_fixture)
{

    RoseXMLWriter * main_xml = exp->getMainXML();
    
    if (exec->wasVisited())
	return;

    exec->visit();

    StpAsmShapeRepVec shapes;
    {
	Workplan * wp = Workplan::find(exec);

	if (wp) {
	    unsigned sz = wp->size_its_elements();
	    unsigned i;
	    
	    main_xml->beginElement ("workplan");
	    stp_product_definition_formation * setup = wp->get_its_setup();

	    show_fixture = (setup == 0);
	    
	    dump_exec_atts(&shapes, main_xml, wp, show_fixture);

	    if (setup) 
		exp->appendRef(main_xml, "setup", setup);
	    
	    if (sz > 0) {
		main_xml->beginAttribute("elements");
		for (i=0; i<sz; i++) {
		    if (i > 0) main_xml->text (" ");		    
		    add_ref_id(main_xml, wp->get_its_elements(i)->getValue());
		}

		main_xml->endAttribute();
	    }
	    
	    main_xml->endElement("workplan");

	    if (setup) 
		dump_setup(exp, setup);
	    
	    for (i=0; i<sz; i++) {
		stp_machining_process_executable * ex =
		    wp->get_its_elements(i)->getValue();
		dump_exec(exp, ex, show_fixture);
	    }

	    dump_shapes(exp, &shapes);
	    
	    return;
	}
    }

    {
	Machining_workingstep * ws = Machining_workingstep::find(exec);
	if (ws) {
	    
	    main_xml->beginElement("workingstep");
	    
	    dump_exec_atts(&shapes, main_xml, ws, show_fixture);

	    stp_machining_operation * op = ws->get_its_operation();
	    stp_axis2_placement_3d * placement = ws->get_toolpath_orientation();
	    
	    exp->appendRef (main_xml, "op", op);
	    exp->appendRef (main_xml, "orientation", placement);
	    
	    main_xml->endElement("workingstep");

	    printf ("dumping op:%p\n",op );
	    
	    dump_operation (exp, op);

	    printf ("Dumping placement:%p\n", placement);
	    dump_placement (exp, main_xml, placement);
	    
	    dump_shapes(exp, &shapes);
	    return;
	}
    }

    {
	Selective * sel = Selective::find(exec);
	if (sel) {
	    unsigned i;
	    main_xml->beginElement("selective");
	    dump_exec_atts(&shapes, main_xml, sel, show_fixture);

	    main_xml->beginAttribute ("elements");

	    unsigned sz = sel->size_its_elements();
	    for (i=0; i<sz; i++) {
		if (i > 0) main_xml->text (" ");
		add_ref_id (main_xml, sel->get_its_elements(i)->getValue());
	    }
	    main_xml->endAttribute();
	    
	    main_xml->endElement("selective");

	    for (i=0; i<sz; i++) {
		dump_exec(exp, sel->get_its_elements(i)->getValue(),
			  show_fixture);
	    }

	    dump_shapes(exp, &shapes);
	    return;
	}
    }

    {
	Frame_definition_workingstep * ws
	    = Frame_definition_workingstep::find(exec);
	if (ws) {
	    printf ("Have FDW\n");
	    main_xml->beginElement("frame_definition_workingstep");

	    add_id(main_xml, ws->getRootObject());
	    main_xml->addAttribute("name", ws->get_its_id());
    
	    main_xml->endElement("frame_definition_workingstep");

	    return;
	}
    }

    {
	Compensation_workingstep * ws
	    = Compensation_workingstep::find(exec);
	if (ws) {
	    main_xml->beginElement("compensation_workingstep");
	    add_id(main_xml, ws->getRootObject());
	    main_xml->addAttribute("name", ws->get_its_id());
	    main_xml->endElement("compensation_workingstep");
	    
	    return;
	}
    }
    
    printf ("resolve_exec: could not find exec: #%d (%s)\n", exec->entity_id(),
	    exec->domain()->name());
    exit (2);
}




void dump_project(
    StixMeshXMLExport * exp,
    Project * proj)
{
    unsigned i,sz;

    stp_machining_workplan * wp = proj->get_main_workplan();

    RoseXMLWriter * main_xml = exp->getMainXML();
    
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

    main_xml->beginElement("project");
    add_ref (main_xml, "wp", wp);

    add_shape_refs(main_xml, "as_is", &as_is);
    add_shape_refs(main_xml, "to_be", &to_be);
    
    main_xml->endElement("project");

    dump_shapes(exp, &as_is);
    dump_shapes(exp, &to_be);
        
    dump_exec (exp, wp, ROSE_TRUE);
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
 * usage: stpfile xmlfile
 */
int NCtoXML(stp2webgl_opts * opts)
{
    /* Disable buffering */
    setvbuf(stdout, 0, _IONBF, 2);

    const char * fname = 0;
    const char * xmlname = 0;


    RoseBoolean write_dir = ROSE_FALSE;    
    const char * out_dir = 0;    

    if (opts->do_split){
	write_dir = ROSE_TRUE;
	fname = opts->srcfile;
    }
    else{
	if (opts->srcfile)
	    fname = opts->srcfile;
    }

    FILE * out =0;
    
    if (!write_dir) {
	if (!xmlname)
	    out = fopen("YourXMLFile.xml", "w");
	else
	   out = fopen (xmlname, "w"); 
    }
    else {
	if (!xmlname)
	    xmlname = "Full_XML_Moldy";

	out_dir = xmlname;
	if (mkdir(out_dir MKDIR_PROT_PARM) && !(DIR_ALREADY_EXISTS)) {
	    printf ("Cannot create directory %s\n", out_dir);
	    exit (2);
	}
	    
	RoseStringObject path = out_dir;
	path.cat ("/index.xml");
	out = fopen(path, "w");
    }

    stix_tag_units(opts->design);

    stix_tag_asms(opts->design);
    stixmesh_resolve_presentation(opts->design);
    ARMpopulate(opts->design);

    ARMCursor cur;
    cur.traverse(opts->design);
    cur.domain(Project::type());

    Project * proj;

    RoseOutputFile xmlfile(out, xmlname? xmlname: "xml file");
    RoseXMLWriter xml(&xmlfile);
    xml.escape_dots = ROSE_FALSE;
    xml.writeHeader();

    StixMeshXMLExport xml_state(&xml, out_dir);
    
    xml.beginElement("project-data");

    StixUnit unit = get_unit(opts->design);
    
    xml.beginAttribute ("unit");
    xml.text (stix_get_unit_name(unit));
    char buff[20];
    sprintf (buff, " %f", stix_get_converted_measure(1., unit, stixunit_m));
    xml.text(buff);
    xml.endAttribute();
    
    ROSE.beginTraversal();

    printf ("Dumping project\n");

    while ((proj = ARM_CAST(Project, cur.next())) != 0) {
	dump_project(&xml_state, proj);
    }   

    ROSE.endTraversal();
    
    xml.endElement("project-data");
    system("pause");

    xml.close();
    xmlfile.flush();
    return 0;
}
